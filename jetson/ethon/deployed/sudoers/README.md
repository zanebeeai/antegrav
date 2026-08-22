# Installed sudo policies

The Jetson has these Ethon-specific policy files installed:

```text
/etc/sudoers.d/ethon-clear-estop
/etc/sudoers.d/ethon-hmi
```

They are root-readable and could not be downloaded through the unprivileged
`jetson` SFTP account during the 2026-08-22 snapshot. A source copy of the HMI
policy is available at `../../ethon-hmi.sudoers`. Capture and review the exact
installed files with privileged access before rebuilding the Jetson.
