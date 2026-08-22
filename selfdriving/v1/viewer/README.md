# Ethon Run Inspector

This local viewer keeps capture data on the operator's computer. It displays the
wide and narrow videos on one timeline and shows the nearest telemetry sample,
signal traces, events, run metadata, and file-health warnings.

## Prepare a downloaded run

From this directory:

```bash
python prepare_run.py ../../../jetson/ethon/data/raw/YYYY-MM-DD/<run_id>
```

This creates `viewer.json` inside the run directory. It does not modify the MP4
or Parquet source files. Telemetry is reduced to 20 Hz and frame diagnostics to
10 Hz for responsive viewing; the original data remains unchanged.

## Open the viewer

```bash
npm install
npm run dev
```

Open the local address printed by the command, select **Open run folder**, and
choose the run directory containing `viewer.json`, both MP4s, and the three
Parquet files.

The wide video is the master playback clock. The narrow video is continuously
corrected against its recorded first-frame timestamp. Scrubbing also updates
both videos and selects the nearest time-aligned telemetry sample.

Run data and generated `viewer.json` files are ignored by Git. Do not deploy a
viewer containing private capture files to a public host.
