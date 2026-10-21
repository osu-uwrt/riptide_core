# VectorNav Workshop

`sensor_config` is a Linux Qt application for inspecting, configuring, and
calibrating a VN-100. It connects to the running robot driver over ROS by
default, so the dashboard can run on an operator computer while the sensor
remains wired to the vehicle. It is built only where Qt5, Eigen and yaml-cpp are
installed; without them `riptide_imu` builds just the `imu` driver. Its fonts, icons,
register list and calibration policy live in `config_tool_assets/`.

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
same physical port locally. Each click in the robot / port box rediscovers drivers or
rescans serial devices (without connecting), then opens the whole list; typing narrows it
to the entries containing the text. The baud box opens its list the same way.

Driver-critical communication, synchronization, binary-output, and reference
frame registers require an expiring maintenance session in ROS mode. Ordinary
reads, monitoring, and calibration capture leave the running output
configuration unchanged. Persistent save sends the flash command without
depending on its delayed acknowledgment, resets the sensor, reconnects at the
effective baud, and reports success only after post-reset readback matches the
dashboard snapshot. This intentionally creates a short IMU-data gap and resets
sensor estimator history.

## Dashboard

The application follows the Nereus viewer's look: View › Theme offers **Timing board**
(charcoal sheets on a black board, the default) and **Heat sheet** (its light twin),
with the ink board across the top, ruled headings, square status chips, and every
number in Barlow TF (`config_tool_assets/fonts/`, SIL OFL, copied from `nereus/content/viewer/fonts`). The
colors are those of `nereus/content/viewer/themes/heat-sheet.yaml` and
`timing-board.yaml`; change them in `src/config_tool/theme.cpp` when those files change.
The window and dock icon (`config_tool_assets/icons/sensor_config-*.png`) is drawn by
`config_tool_assets/icons/make_icons.py`; regenerate it with `uv run --with cairosvg python
riptide_imu/config_tool_assets/icons/make_icons.py` from `src/riptide_core`.

The board holds the connection (mode switch, robot or port, baud, Connect) and chips for
the sensor's output rate, maintenance, and the count of staged values. The command bar below it
reads, applies, discards, imports, exports and saves to flash, and opens maintenance
and calibration. While connected it also shows the **line load**: the share of serial port
1's baud rate that the sensor's own output takes (ASCII async output, registers 6 and 7,
plus every binary output routed to port 1, registers 75-77, at 10 bits per byte), with
the staged configuration's load beside it. Applying or saving a configuration over 100%
is refused, and the tooltip breaks the load down by output. Panels: the register list on the left, the register inspector in the
middle above Signals / Calibration / Protocol log, and Attitude above Live data on the
right. Splitter positions and the window are remembered per user; View › Reset layout
restores them. `Ctrl+F` focuses register search, `F5` reads the selected register,
`Ctrl+Return` applies staged values and `Ctrl+K` opens calibration. Every click on a register
while connected reads it, including a click on the one already selected, so the device
column is current. Registers with a staged change are not read over, and register 80 is
never read this way because reading it resets its integration interval; **Read** (F5)
reads either on request. Registers and fields carry the manual's names; enumerated fields are menus, and the
binary output registers are edited as their port, rate and a checkbox per output field,
with each field's size and the resulting load. Staged registers are highlighted in the
list; every register keeps an editable raw protocol representation for diagnosis and
forward compatibility.

Signals plots attitude (roll, pitch, unwrapped yaw), angular rate, acceleration or the
magnetic field over 20 s to 30 min; the axis buttons hide or show each line and the scale
follows the lines shown. Live data adds drift while still: the least-squares slope of
roll, pitch and yaw over the last 60 s without motion (any rate above 0.05 rad/s restarts
it), and the total since **Restart drift measurement**. It also separates the sensor's
output rate (from its registers) from how often this view updates: over local serial the
view polls registers every 50 ms, so it updates at about 20 Hz whatever the sensor sends.
The protocol log timestamps each line and **Export log…** saves it as text.

The attitude view draws a model of the VN-100 Rugged built from the manual's mechanical
drawing (section 2.4; `src/config_tool/sensor_model.cpp`): the red case with its
chamfered connector end, the 10-pin Harwin M80 connector on -X, both mounting tabs with
their screw and dowel holes, and the printed mark and axis legend on top. The body axes
(X red, Y green, Z blue) start at the sensor origin on the bottom face, as the manual
places it, and pass into the case. A ground grid with compass letters, a level ring and
the case's shadow complete the scene. The model is drawn by a small software depth
buffer, so it renders identically offscreen. Drag to orbit, scroll to zoom, double-click to
reset; **Level** looks side-on to read tilt against the ring and **Follow heading** keeps
the camera behind the sensor. Orbiting by hand switches the preset back to **3D**, so any
preset is one click away. Roll, pitch and yaw are shown as the VN-100 reports them
(3-2-1 from NED). Directly over serial the quaternion is relative to NED; over ROS the
driver publishes it relative to ENU, and the view converts back.

## Guided calibration

**Calibrate…** opens the calibration in its own window. Each step shows a drawing of
what to do and checks the capture as it happens, so the operator never starts or stops
recording by hand:

1. **Prepare** checks that calibration samples arrive, reads registers 23, 25, 26, 43
   and 84, watches the temperature trend (steady below 0.15 °C/min, changing above
   0.25 °C/min, each only after holding 15 s), and lets you choose the steps to run. It
   also picks the directions: the sensor's own axes, or a robot config (see below).
2. **Gyro bias**: set the sensor down and step away. A 30 s hold starts once the
   sensor is still and starts over if anything moves.
3. **Accelerometer**: a disc shows which end of the sensor is low. Tip it into each of
   nine rings (flat, each end and each corner at 45°); a ring fills after 2.5 s of
   stillness. A tilted ring only counts from 40°, which keeps every axis span above
   the 2 m/s² gate.
4. **Magnetometer**: turn the level sensor slowly through a full circle; a ring of
   10° sectors fills from the integrated gyro rate, with live warnings for tilt and turn
   rate. Coverage is the span between the furthest headings reached, starting from zero;
   the gyro's bias, measured while still before the capture, is removed first.
5. **Gravity report**: a 10 s hands-off hold as mounted (report only). It gives the
   sensor's roll and pitch against gravity as the VN-100 reports them; with a robot config,
   also the robot's roll and pitch (ROS convention) and the IMU `pose:` line that would make
   the robot read level. Both carry the warning that they are only valid if the robot was
   level during the capture, and the copied line keeps it as a YAML comment.
6. **Review** stages every passed result into the register workspace and exports the
   report. Staging never writes the sensor: apply, then save to flash, as for any
   register.

Each step page separates **Checks** (measured against a limit, all OK means the capture
can be trusted) from the **Result** (what was measured, its 3σ uncertainty, and the
register bias it would stage, before and after). Whether a result is staged or already on
the sensor is read off the registers each time, so discarding staged values shows at once;
**Stage latest results** on the Calibration tab stages them again.

A robot description config with an `imu` entry whose `pose` is
`[x, y, z, roll, pitch, yaw]` (`riptide_descriptions/config/talos.yaml`,
`mercury_descriptions/.../mercury_xacro_frames.yaml`, or any mapping with "imu" in its
key) turns the poses into the robot's: "front down", "right side down", with the robot's
sides on the disc and the sensor drawn as mounted. The imu_link axes are taken as the
sensor's printed axes and base_link as ROS standard (X forward, Y left, Z up). Configs
you pick are remembered.

Each step reports its checks against `config_tool_assets/config/calibration_policy.json` and, when one
fails, what to change. The live checks only guide the capture; pass or fail comes from
the same estimators as before. Holds are timed from the samples' own timestamps. Demo
data moves continuously, so the holds never settle in demo mode.

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
It reports tilt relative to sensor +Z, and roll and pitch, and cannot observe yaw. It
can only separate mounting error from a tilted robot when the robot is known to be level. Its vector is pose-dependent and must not be
installed as a fixed EKF subtraction. The existing IMU publisher and EKF files
are not modified; acceleration continues to contain gravity exactly once.

The magnetometer procedure fits a horizontal rotation circle in the plane found
from measured gravity. It updates only the observable in-plane hard-iron term
and preserves the unobservable normal component and soft-iron matrix.

## Register and file handling

`config_tool_assets/config/registers.json` describes the VN-100 firmware 3.0 register set from the
bundled manual, with the manual's register and field names. Unsupported XML entries
survive import/export but cannot be written. Register 5, 6, and 7 retain both
serial-port values used by VectorNav Control Center files. Connection-affecting
changes require confirmation.

The sensor stores floating-point values as float32 and echoes about six significant
digits, so a write is verified when the readback agrees to that precision (decimals within
1e-5 relative, integers and hex words exactly). The serial transport and the driver's
`vectornav/config` service use the same comparison.

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
colcon test-result --test-result-base build/riptide_imu --verbose
```

The deterministic suite covers robust gyro estimation, motion and temperature
rejection, reference-frame-aware compensation, repeat application, singular
matrices, arbitrary-pose accelerometer bias and gravity magnitude, degenerate
pose rejection, planar magnetic calibration, gravity-report conventions, XML
round trips, unknown entries, and register validation. `test_calibration_guide`
covers the wizard's stillness, pose and heading checks, including that the
shallowest accepted poses still pass the accelerometer gate, robot-frame poses, the
gravity roll / pitch and mount correction, and the staged / applied tracking.
`test_sensor_tools` covers the line-load estimate against the robot's real output
configuration, drift measurement, and reading talos- and mercury-style robot configs.
`test_dashboard` drives each wizard step with synthetic sensor data, checks auto-read,
the binary output editor and its load guard, re-staging after discard, the orbit preset
and the filtering target list, and writes screenshots of both themes to
`/tmp/vn_dashboard_*.png` and `/tmp/vn_wizard_*.png`.

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
existing value and verify readback. Flash persistence is checked by hand with
**Save to flash**, which already resets and reads back. Bias capture is done with the
guided calibration on the robot.

### Calibration simulation

`test_calibration_simulation` runs every deterministic calibration scenario without
a sensor. Simulation inputs, ground truth, noise, poses and expected gates are
stored in `test/config_tool/calibration_simulation.json`, next to the simulator
itself; the test loads the same `calibration_policy.json` the dashboard uses and
fails when an observed quality decision differs from the expectation.

The simulator adapts the Nereus physics pattern: world-frame gravity, magnetic
field, and Earth rate are rotated into the sensor frame before sensor bias,
correlated noise, thermal drift, motion, or magnetic disturbance are added.
The scenarios cover nominal recovery, gyro motion and warm-up, insufficient
accelerometer tilt, motion during pose changes, incomplete magnetic coverage,
magnetic-plane tilt, magnetic interference, and a moving gravity capture.

## Feature-to-test matrix

| Feature | Automated without sensor | Hardware acceptance |
|---|---|---|
| Calibration math and quality gates | `test_calibration`, `test_calibration_simulation` | Held-out stationary poses and repeat runs |
| Guided calibration | `test_calibration_guide`, `test_dashboard` wizard flows | Each step on the robot over ROS and serial |
| Compensation/frame conventions | Matrix, rotation, idempotence fixtures | Readback plus existing driver comparison |
| Control Center XML | `test_configuration` round trips | Windows bidirectional import/export |
| Register validation | Schema and protocol-field tests | Every supported register read; writable volatile round trip |
| UI and live monitor | `demo:=true`, offscreen smoke launch | Live plots, stale state, reconnect and recording |
| Line load and output editor | `test_sensor_tools`, `test_dashboard` | Load matches the driver's packet rate |
| Drift readout | `test_sensor_tools` | A still sensor's yaw drift against a reference |
| Robot config directions | `test_calibration_guide`, `test_sensor_tools`, `test_dashboard` | Poses read correctly on Talos |
| Volatile restoration | Emulator/manual workflow | Snapshot, change, restore, readback |
| Flash persistence | State model/manual workflow | Explicit opt-in recovery procedure above |

Sensor-dependent claims remain unverified until those hardware checks pass.
