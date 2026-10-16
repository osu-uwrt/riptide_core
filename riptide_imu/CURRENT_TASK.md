# Current task

Goal: Validate the VN-100 configuration dashboard over direct serial and the
running ROS driver, preserving and restoring sensor configuration.

Status: Hardware register reads, live telemetry, same-value writes for every
persistent writable schema register, ROS maintenance gating, flash save/reset,
and temporary 460800 to 230400 to 460800 baud transition have passed on
VN-100T-CR serial 0100059698. The final snapshot matched all 30 persistent
writable pre-test values. The original JSON and Control Center XML backups are
in the local `hardware_backups/` directory and must remain uncommitted.

The first assisted bias attempt produced no calibration samples because the
runner targeted `/talos` while the driver was in the root namespace. No
coefficients were applied or saved. The runner now validates ROS endpoint
discovery before launch, the dashboard shows and exports per-step sample
counts, and gravity capture has a buffer separate from accelerometer fitting.

Remaining: rerun the assisted bias suite against the discovered root endpoint;
Windows Control Center bidirectional XML check.
