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

Open the local address printed by the command. The newest locally downloaded
run opens automatically, and the **Local run** dropdown switches between every
indexed run without a file picker. `npm run dev` refreshes the index before the
viewer starts. While it is already running, refresh the list with:

```bash
npm run index-runs
```

**Open run folder** remains available for an unindexed directory containing
`viewer.json`, both MP4s, and the three Parquet files.

The wide video is the master playback clock. The narrow video is continuously
corrected against its recorded first-frame timestamp. Scrubbing also updates
both videos and selects the nearest time-aligned telemetry sample.

The local index uses filesystem links rather than copying the MP4 files. Run
data, generated `viewer.json` files, and the index are ignored by Git. Do not
deploy a viewer containing private capture files to a public host.
