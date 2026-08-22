# Model artifacts

The deployed model artifacts remain on the Jetson under
`/home/jetson/ethon/models` and are intentionally not committed to normal Git.
At the 2026-08-22 snapshot they occupied approximately 767 MB, and several
individual files exceeded GitHub's standard 100 MB file limit.

`REMOTE_MANIFEST.sha256` records the identity of every deployed model artifact.
Store restorable copies in an artifact store, release attachment, or Git LFS
before deleting or replacing anything on the Jetson.

The active perception default in `birdseye_fusion.py` and `ethon_capture.py` is:

```text
/home/jetson/ethon/models/ethon_v1.engine
```

