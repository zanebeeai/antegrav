'use client';

import { ChangeEvent, useEffect, useMemo, useRef, useState } from 'react';

type Point = {
  t: number;
  speed?: number | null;
  steer?: number | null;
  steer_rate?: number | null;
  target?: number | null;
  pedal?: number | null;
  latency?: number | null;
  ctre_ok?: boolean | null;
  voltage?: number | null;
  drive_current?: number | null;
  gps_lat?: number | null;
  gps_lon?: number | null;
  gps_heading?: number | null;
  gps_fix?: number | null;
  mode?: string | null;
  estop?: boolean | null;
  faults?: string | null;
};

type RunData = {
  run_id: string;
  metadata: Record<string, unknown>;
  health: { ok: boolean; warnings: string[]; source_rows: Record<string, number> };
  timeline: { duration_s: number };
  videos: {
    wide: { file: string; valid: boolean; start_s: number; size_bytes: number };
    narrow: { file: string; valid: boolean; start_s: number; size_bytes: number };
  };
  telemetry: Point[];
  frames: Array<Record<string, unknown> & { t: number }>;
  events: Array<{ t: number; type: string; value?: string; notes?: string }>;
};

type VideoUrls = { wide?: string; narrow?: string };

const fmt = (value: number | null | undefined, digits = 2) =>
  typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';

function nearest(points: Point[], time: number) {
  if (!points.length) return undefined;
  let low = 0;
  let high = points.length - 1;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (points[mid].t < time) low = mid + 1;
    else high = mid;
  }
  const left = points[Math.max(0, low - 1)];
  const right = points[low];
  return Math.abs(left.t - time) <= Math.abs(right.t - time) ? left : right;
}

function TelemetryPlot({ points, time }: { points: Point[]; time: number }) {
  const width = 920;
  const height = 180;
  const duration = Math.max(points.at(-1)?.t || 1, 1);
  const lines = useMemo(() => {
    const make = (key: keyof Point, color: string, scale: number) => ({
      color,
      path: points
        .filter((point) => typeof point[key] === 'number')
        .map((point, index) => {
          const x = (point.t / duration) * width;
          const raw = Number(point[key]);
          const y = height / 2 - Math.max(-1, Math.min(1, raw / scale)) * (height * 0.42);
          return `${index ? 'L' : 'M'}${x.toFixed(1)},${y.toFixed(1)}`;
        }).join(' '),
    });
    return [
      make('steer', '#ffb454', 1.5),
      make('speed', '#60a5fa', 3),
      make('pedal', '#4ade80', 1),
    ];
  }, [points, duration]);
  const cursor = Math.max(0, Math.min(width, (time / duration) * width));

  return (
    <div className="plot" aria-label="Steering, speed, and pedal plot">
      <svg viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
        <line className="zero" x1="0" x2={width} y1={height / 2} y2={height / 2} />
        {lines.map((line) => <path key={line.color} d={line.path} stroke={line.color} />)}
        <line className="cursor" x1={cursor} x2={cursor} y1="0" y2={height} />
      </svg>
      <div className="legend"><span className="steer">Steering</span><span className="speed">Speed</span><span className="pedal">Pedal</span></div>
    </div>
  );
}

export default function Home() {
  const [run, setRun] = useState<RunData | null>(null);
  const [loadError, setLoadError] = useState('');
  const [urls, setUrls] = useState<VideoUrls>({});
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [rate, setRate] = useState(1);
  const wideRef = useRef<HTMLVideoElement>(null);
  const narrowRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    fetch('/run/viewer.json')
      .then((response) => {
        if (!response.ok) throw new Error('No prepared run is loaded');
        return response.json();
      })
      .then((value: RunData) => setRun(value))
      .catch(() => setLoadError('Open a prepared run folder to begin.'));
  }, []);

  useEffect(() => () => {
    Object.values(urls).forEach((url) => url && URL.revokeObjectURL(url));
  }, [urls]);

  const current = useMemo(() => nearest(run?.telemetry || [], time), [run, time]);
  const duration = Math.max(run?.timeline.duration_s || 0, 0.01);

  const seek = (next: number) => {
    const bounded = Math.max(0, Math.min(duration, next));
    setTime(bounded);
    const videos: Array<[HTMLVideoElement | null, number]> = [
      [wideRef.current, run?.videos.wide.start_s || 0],
      [narrowRef.current, run?.videos.narrow.start_s || 0],
    ];
    videos.forEach(([video, start]) => {
      if (!video || !Number.isFinite(video.duration)) return;
      const target = Math.max(0, Math.min(video.duration, bounded - start));
      if (Math.abs(video.currentTime - target) > 0.08) video.currentTime = target;
    });
  };

  const togglePlay = async () => {
    const videos = [wideRef.current, narrowRef.current].filter(Boolean) as HTMLVideoElement[];
    if (playing) {
      videos.forEach((video) => video.pause());
      setPlaying(false);
      return;
    }
    videos.forEach((video) => { video.playbackRate = rate; });
    await Promise.allSettled(videos.map((video) => video.play()));
    setPlaying(true);
  };

  const onTimeUpdate = () => {
    const video = wideRef.current;
    if (!video || !run) return;
    const master = video.currentTime + run.videos.wide.start_s;
    setTime(master);
    const narrow = narrowRef.current;
    if (narrow && !narrow.paused) {
      const target = master - run.videos.narrow.start_s;
      if (target >= 0 && Math.abs(narrow.currentTime - target) > 0.12) narrow.currentTime = target;
    }
  };

  const openFolder = async (event: ChangeEvent<HTMLInputElement>) => {
    const files = Array.from(event.target.files || []);
    const byName = new Map(files.map((file) => [file.name, file]));
    const manifest = byName.get('viewer.json');
    if (!manifest) {
      setLoadError('This folder has no viewer.json. Run prepare_run.py on it first.');
      return;
    }
    try {
      const data = JSON.parse(await manifest.text()) as RunData;
      Object.values(urls).forEach((url) => url && URL.revokeObjectURL(url));
      const wide = byName.get(data.videos.wide.file);
      const narrow = byName.get(data.videos.narrow.file);
      setUrls({
        wide: wide && wide.size ? URL.createObjectURL(wide) : undefined,
        narrow: narrow && narrow.size ? URL.createObjectURL(narrow) : undefined,
      });
      setRun(data);
      setLoadError('');
      setTime(0);
      setPlaying(false);
    } catch (error) {
      setLoadError(`Could not open viewer.json: ${String(error)}`);
    }
  };

  return (
    <main>
      <header className="topbar">
        <div className="brand"><span className="mark">E</span><div><strong>Ethon Run Inspector</strong><small>Time-synchronized capture review</small></div></div>
        <label className="open-button">Open run folder<input type="file" multiple {...({ webkitdirectory: '', directory: '' } as object)} onChange={openFolder} /></label>
      </header>

      <section className="runbar">
        <div><span className="eyebrow">RUN</span><h1>{run?.run_id || 'No run loaded'}</h1></div>
        <div className={`health ${run?.health.ok ? 'ok' : 'bad'}`}><span />{run?.health.ok ? 'Complete' : 'Needs attention'}</div>
        <dl>
          <div><dt>Track</dt><dd>{String(run?.metadata.track_name || '—')}</dd></div>
          <div><dt>Driver</dt><dd>{String(run?.metadata.driver_identifier || '—')}</dd></div>
          <div><dt>Direction</dt><dd>{String(run?.metadata.track_direction || '—')}</dd></div>
          <div><dt>Duration</dt><dd>{duration.toFixed(1)} s</dd></div>
        </dl>
      </section>

      {(loadError || (run && !run.health.ok)) && (
        <section className="alert">
          <strong>{loadError || 'This run did not finalize cleanly.'}</strong>
          {run?.health.warnings.map((warning) => <span key={warning}>{warning}</span>)}
        </section>
      )}

      <section className="camera-grid">
        {(['wide', 'narrow'] as const).map((name) => {
          const source = urls[name];
          const video = run?.videos[name];
          return (
            <article className="camera" key={name}>
              <div className="camera-title"><span>FRONT {name.toUpperCase()}</span><small>{video?.valid ? 'H.264 MP4' : 'NO PLAYABLE VIDEO'}</small></div>
              {source ? (
                <video ref={name === 'wide' ? wideRef : narrowRef} src={source} muted playsInline onTimeUpdate={name === 'wide' ? onTimeUpdate : undefined} onEnded={() => setPlaying(false)} />
              ) : (
                <div className="video-empty"><span>{video?.size_bytes === 0 ? '0 byte file' : 'Choose the run folder to attach video'}</span></div>
              )}
            </article>
          );
        })}
      </section>

      <section className="transport">
        <button onClick={togglePlay} disabled={!urls.wide && !urls.narrow}>{playing ? 'Pause' : 'Play'}</button>
        <span className="time">{time.toFixed(2)} / {duration.toFixed(2)} s</span>
        <input aria-label="Run time" type="range" min="0" max={duration} step="0.01" value={Math.min(time, duration)} onChange={(event) => seek(Number(event.target.value))} />
        <select aria-label="Playback speed" value={rate} onChange={(event) => { const next = Number(event.target.value); setRate(next); [wideRef.current, narrowRef.current].forEach((video) => { if (video) video.playbackRate = next; }); }}>
          <option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1">1×</option><option value="2">2×</option>
        </select>
      </section>

      <section className="telemetry-grid">
        <div className="metric primary"><span>Vehicle speed</span><strong>{fmt(current?.speed)} <small>m/s</small></strong></div>
        <div className="metric"><span>Steering shaft</span><strong>{fmt(current?.steer, 3)} <small>rad</small></strong></div>
        <div className="metric"><span>Pedal</span><strong>{fmt(typeof current?.pedal === 'number' ? current.pedal * 100 : current?.pedal, 1)} <small>%</small></strong></div>
        <div className="metric"><span>CTRE latency</span><strong>{fmt(current?.latency, 1)} <small>ms</small></strong></div>
        <div className="metric"><span>Supply</span><strong>{fmt(current?.voltage, 1)} <small>V</small></strong></div>
        <div className="metric"><span>GPS</span><strong>{current?.gps_fix ? `Fix ${current.gps_fix}` : 'No fix'}</strong></div>
      </section>

      <section className="analysis-grid">
        <article className="panel"><div className="panel-head"><div><span className="eyebrow">SIGNALS</span><h2>Drive trace</h2></div><small>{run?.telemetry.length || 0} display samples</small></div><TelemetryPlot points={run?.telemetry || []} time={time} /></article>
        <article className="panel"><div className="panel-head"><div><span className="eyebrow">ANNOTATIONS</span><h2>Events</h2></div><small>{run?.events.length || 0}</small></div><div className="event-list">{run?.events.length ? run.events.map((event, index) => <button key={`${event.t}-${index}`} onClick={() => seek(event.t)}><time>{event.t.toFixed(2)} s</time><span><strong>{event.type}</strong><small>{event.value || event.notes || '—'}</small></span></button>) : <p>No readable events.</p>}</div></article>
      </section>

      <footer><span>Prepare a run: <code>python prepare_run.py &lt;run-directory&gt;</code></span><span>Data stays on this computer.</span></footer>
    </main>
  );
}
