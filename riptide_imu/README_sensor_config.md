# VectorNav Workshop

`sensor_config` is a Linux Qt application for inspecting, configuring, and
calibrating a VN-100. Stop the normal `imu` node before opening the same serial
port: configuration mode requires exclusive ownership.

```bash
ros2 run riptide_imu sensor_config --ros-args -p imu_port:=/dev/ttyUSB0
ros2 launch riptide_imu sensor_config.launch.py imu_port:=/dev/ttyUSB0
ros2 launch riptide_imu sensor_config.launch.py demo:=true
```

The optional launch arguments are `baud`, `config_file`, and `demo`. An imported
file only stages values. Register application and flash persistence are separate
actions, and every volatile write is read back.

The `imu_port` and `baud` arguments initialize the connection bar. Its port
field is an editable dropdown populated from `/dev/serial/by-id`,
`/dev/ttyUSB*`, `/dev/ttyACM*`, and `/dev/ttyTHS*`. Select a detected device or
enter any serial path, then reconnect without relaunching the application.

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
scope. Calibration collection never writes the sensor. Skipping or clearing a
step leaves its registers unchanged.

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
