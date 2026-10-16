# VectorNav Workshop

`sensor_config` is a Linux Qt application for inspecting, configuring, and
calibrating a VN-100. It connects to the running robot driver over ROS by
default, so the dashboard can run on an operator computer while the sensor
remains wired to the vehicle.

```bash
ros2 run riptide_imu sensor_config
ros2 launch riptide_imu sensor_config.launch.py
ros2 launch riptide_imu sensor_config.launch.py remote_namespace:=/talos
ros2 launch riptide_imu sensor_config.launch.py demo:=true
```

The optional launch arguments are `connection_mode`, `remote_namespace`,
`baud`, `config_file`, and `demo`. `connection_mode` defaults to `ros` and an
empty namespace enables discovery. An imported file only stages values.
Register application and flash persistence are separate actions, and every
volatile write is read back.

The connection bar switches between **Robot over ROS**, **Local serial**, and
**Demo** without relaunching. ROS mode discovers each `vectornav/config`
service and uses the matching `vectornav/imu` topic. Local mode provides an
editable dropdown populated from `/dev/serial/by-id`, `/dev/ttyUSB*`,
`/dev/ttyACM*`, and `/dev/ttyTHS*`; stop the robot driver before claiming the
same physical port locally.

Driver-critical communication, synchronization, binary-output, and reference
frame registers require an expiring maintenance session in ROS mode. Ordinary
reads, monitoring, and calibration capture leave the running output
configuration unchanged. Persistent save sends the flash command without
depending on its delayed acknowledgement, resets the sensor, reconnects at the
effective baud, and reports success only after post-reset readback matches the
dashboard snapshot. This intentionally creates a short IMU-data gap and resets
sensor estimator history.

## Dashboard

The application uses a dark, dockable workbench designed for simultaneous
configuration and monitoring. The grouped register tree and structured
device/staged-value inspector remain central, live attitude and numeric sensor
data occupy the right dock, and plots, calibration, and protocol diagnostics
share tabbed lower docks. Use the **View** menu to show hidden panels or reset
the saved layout. Window geometry and dock placement are stored per user.

The main toolbar exposes read, apply, discard, XML import/export, persistent
save, telemetry recording, and replay actions. `Ctrl+F` focuses register search,
`F5` reads the selected register, and `Ctrl+Return` applies validated staged
values. Structured enum and matrix/vector fields retain an editable raw
protocol representation for diagnosis and forward compatibility.

Package-owned C++ code is separated by purpose: the ROS driver is under
`src/driver` and `include/riptide_imu/driver`, while the configuration application
is under `src/config_tool` and `include/riptide_imu/config_tool`.

## Calibration signal path and safety

The firmware path documented by the bundled VN-100 manual is:

```
MEMS -> factory calibration -> user C(x-B) -> reference-frame rotation
     -> register 54 / uncompensated outputs -> filter bias compensation
     -> compensated outputs used by the existing driver
```

The guided calibration samples register 54 through VNProglib. Its residuals are
therefore in the configured output frame. When staging user compensation, the
application transforms the residual through the inverse reference rotation and
the inverse existing compensation matrix:

```
B_new = B_old + C^-1 R^-1 residual_output
```

This preserves existing scale/alignment coefficients and makes a second
calibration correct only the remaining residual. A singular matrix blocks the
operation. Gyro compensation also requires register 43 to have been read and be
zero; a nonzero startup filter bias blocks staging because applying both without
firmware-specific validation can double-correct angular rate.

No procedure assumes a level surface. Accelerometer calibration jointly fits a
bias and sphere radius from varied stationary poses. The radius is reported as
the measured magnitude under the retained scale calibration, not exact local
gravity. Poor pose coverage or bias/radius coupling fails the quality gate.

The gravity report states `gravity_vector_sensor = -mean_stationary_specific_force`.
It reports tilt relative to sensor +Z and cannot observe yaw or isolate
sensor-to-robot mounting error. Its vector is pose-dependent and must not be
installed as a fixed EKF subtraction. The existing IMU publisher and EKF files
are not modified; acceleration continues to contain gravity exactly once.

The magnetometer procedure fits a horizontal rotation circle in the plane found
from measured gravity. It updates only the observable in-plane hard-iron term
and preserves the unobservable normal component and soft-iron matrix.

## Register and file handling

`config/registers.json` describes the VN-100 firmware 3.0 register set from the
bundled manual. Unsupported XML entries survive import/export but cannot be
written. Register 5, 6, and 7 retain both serial-port values used by VectorNav
Control Center files. Connection-affecting changes require confirmation.

Persistent save writes the sensor's complete active state. The application
requires all staged values to be applied and verified first, then previews this
scope. In ROS mode the driver performs save, reset, baud-aware reconnect, and
readback as one idempotent operation. Calibration collection polls register 54
through the driver and never uses the EKF-facing angular velocity as a bias
sample. Skipping or clearing a step leaves its registers unchanged.

## Validation

Run the package tests with:

```bash
colcon test --packages-select riptide_imu --event-handlers console_direct+
colcon test-result --verbose
```

The deterministic suite covers robust gyro estimation, motion and temperature
rejection, reference-frame-aware compensation, repeat application, singular
matrices, arbitrary-pose accelerometer bias and gravity magnitude, degenerate
pose rejection, planar magnetic calibration, gravity-report conventions, XML
round trips, unknown entries, and register validation.

Before using real hardware, capture the current Control Center XML and read all
writable registers. Default acceptance runs may apply volatile values only and
must restore/read back that snapshot. Flash persistence testing is a separate
manual opt-in: save a recovery XML and journal outside the robot, record both
active and reboot-persistent state, apply the test value, save and reboot,
verify it, restore the saved state, save and reboot again, then restore any
previously active-only values without saving. Abort if the initial snapshot is
incomplete. Power loss can prevent automatic restoration, and counters or
estimator history are not restorable configuration.

Hardware acceptance must additionally verify after restarting `imu` that its
quaternion transform, angular-rate convention, acceleration convention, output
rate, synchronization, and EKF gravity-removal behavior match the pre-test
baseline. XML compatibility with the current Windows Control Center must be
checked using import and export in both directions; the supplied example proves
the format subset used here but cannot prove every Control Center version.

The opt-in Qt hardware suite uses the same transports as the dashboard. Keep the
pre-test journal outside version control and run direct mode only while the
driver is stopped:

```bash
VN_HARDWARE_PORT=/dev/ttyUSB0 \
VN_HARDWARE_SNAPSHOT=/path/to/pretest.json \
test_hardware_integration -platform offscreen

VN_HARDWARE_ROS=1 VN_HARDWARE_NAMESPACE=/talos \
VN_HARDWARE_SNAPSHOT=/path/to/pretest.json \
test_hardware_integration -platform offscreen
```

Add `VN_HARDWARE_SCHEMA=/path/to/registers.json` and either
`VN_HARDWARE_DIRECT_WRITES=1` or `VN_HARDWARE_WRITES=1` only after the recovery
snapshot is complete. These modes write each persistent writable register's
existing value and verify readback. `hardware_acceptance.py --flash-persistence`
is the separate journaled save/reset/restore test. Bias capture is a later,
operator-assisted acceptance stage.

Run the operator-assisted full bias suite with:

```bash
ros2 run riptide_imu vn_bias_suite \
  --report ~/vn100_bias_report.json
```

The script discovers the active `*/vectornav/config` service and refuses to
start if no driver is available or the requested namespace is wrong. Pass
`--remote-namespace /talos` only when the driver is actually namespaced under
`/talos`. The script launches the dashboard, gives pose-by-pose instructions, never
applies or saves coefficients, validates every exported quality gate, and writes
`~/vn100_bias_report_suite_result.json` with the overall pass/fail result.

## Feature-to-test matrix

| Feature | Automated without sensor | Hardware acceptance |
|---|---|---|
| Calibration math and quality gates | `test_calibration` | Held-out stationary poses and repeat runs |
| Compensation/frame conventions | Matrix, rotation, idempotence fixtures | Readback plus existing driver comparison |
| Control Center XML | `test_configuration` round trips | Windows bidirectional import/export |
| Register validation | Schema and protocol-field tests | Every supported register read; writable volatile round trip |
| UI and live monitor | `demo:=true`, offscreen smoke launch | Live plots, stale state, reconnect and recording |
| Volatile restoration | Emulator/manual workflow | Snapshot, change, restore, readback |
| Flash persistence | State model/manual workflow | Explicit opt-in recovery procedure above |

Sensor-dependent claims remain unverified until those hardware checks pass.
