# riptide_navigation

Navigation estimator for Talos. It fuses the VectorNav IMU, the FOG, the DVL and the depth sensor into
the vehicle's odometry, acceleration and `odom -> talos/base_link` transform. It replaces the
`robot_localization` EKF on the vehicle.

The estimator is a generated ROS 2 Simulink model, `talos_navigation`, built around Simulink's
Extended Kalman Filter block. A small C++ node, `talos_navigation_overseer`, supervises it. The model is
built and published by `riptide_simulink`; the vehicle runs the generated C++ and
does not need MATLAB.

## State and filter

The 19-state layout is:

```
[position_odom_xyz, quaternion_body_to_odom_wxyz,
 velocity_body_xyz, angular_velocity_body_xyz, acceleration_body_xyz,
 accelerometer_bias_body_xyz]
```

- The quaternion is normalized after every update and its covariance is projected through the
  normalization Jacobian before publication.
- IMU tilt is fused as the gravity direction in the body frame, so yaw is excluded and there is no Euler
  singularity. Predicted heading is preserved across each gravity correction, including through
  vertical attitudes and full flips.
- Process noise is a continuous-time covariance density in units per second. Attitude entries are body
  angular-error densities, mapped through the quaternion tangent Jacobian at every prediction.
- Measurement covariance multipliers and a covariance floor are available for tuning. They apply even
  when a sensor publishes its own covariance.
- By default the IMU acceleration update is a consider update: it corrects the accelerometer bias
  but keeps position and attitude out of the acceleration residual.
- Measurements are queued per sensor and sorted by stamp. A late sample restores a checkpoint and replays
  the samples after it, within a configurable window. Publication extrapolates to the ROS clock without
  advancing the measurement-time filter, so paused simulated time produces no new prediction.

## Interfaces

All names are relative to the vehicle namespace (`/talos`).

| Direction | Name | Type |
|---|---|---|
| in | `vectornav/imu` | `sensor_msgs/Imu` |
| in | `gyro/twist` | `geometry_msgs/TwistWithCovarianceStamped` |
| in | `dvl_twist` | `geometry_msgs/TwistWithCovarianceStamped` |
| in | `depth/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| in | `set_pose` | `geometry_msgs/PoseWithCovarianceStamped`, and a `robot_localization/srv/SetPose` service |
| out | `odometry/filtered` | `nav_msgs/Odometry`, twist in the body frame |
| out | `accel/filtered` | `geometry_msgs/AccelWithCovarianceStamped` |
| out | `/tf` | `odom -> talos/base_link` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

The overseer exists because R2023a has no TF or service-server blocks. It does the following:

- loads the model's parameters and reloads them on `reload_model_parameters`;
- reads the IMU, FOG and DVL mounting transforms from TF and passes them to the model;
- serves `set_pose`;
- turns the model's numeric diagnostics (`navigation/diagnostics`) into `/diagnostics`, with a warning when
  heading uncertainty or the unpublished position offset gets large or the clock stalls.

## Build

The model is built from source by `riptide_simulink`, then the package is built from the workspace root:

```bash
ros2 run riptide_simulink model_manager.py build talos_navigation --source local
cd ~/osu-uwrt/development/software
colcon build --packages-select riptide_navigation
source install/setup.bash
```

`--source local` generates from `models/talos_navigation/talos_navigation.slx`. Without MATLAB, use
`model_manager.py download talos_navigation` to fetch the published build instead. The model manager stages
the x86_64 package in the development workspace and the arm64 sources in the release workspace for the robot.
`models/simulink.yaml` is the model's `riptide_simulink` manifest: its setup function, the GitHub release
channel, and the list of custom message packages, which is empty because the model uses only standard
ROS messages.

## Launch

```bash
ros2 launch riptide_navigation navigation.launch.py robot:=talos
```

This starts the rest of the `riptide_hardware2` navigation stack with its own EKF disabled, then the model and
the overseer. Add `use_sim_time:=true` to follow a simulated clock. `mode:=replacement` is accepted so
existing launch files keep working; it is the only mode.

The launch file refuses to start if the config fails validation or the generated `talos_navigation` package is
not installed.

## Configuration

`config/talos_navigation.yaml` holds every setting, grouped as `frames`, `build`, `timing`, `sensor_axes`,
`estimator`, `gates` and `diagnostics`. Each entry has a comment with its meaning and units.

- `config/parameter_schema.json` defines every parameter's type, range, default and consumer (model,
  overseer, or build). The launch file validates the config against it and splits it into the model's and the
  overseer's ROS parameters. The model's parameter blocks are generated from the same file.
- `build` values (rates and buffer sizes) are baked into the generated code. Changing one means rebuilding the
  model; the launch file only checks the other values are consistent with them.
- Each sensor has a 15-element mask in `robot_localization` order that picks which fields are fused.
- Pass `navigation_config:=/path/to/file.yaml` to launch with a different config.
- `config/diagnostic_keys.json` lists the model's numeric diagnostics in order. It has to stay in step with
  `matlab/talos_nav_diag.m`, and the test suite checks that.

## Regenerating the model

`models/talos_navigation/talos_navigation.slx` is produced by `models/make_talos_navigation.m` (MATLAB R2023a
with ROS Toolbox, Simulink and Embedded Coder), from the functions in `matlab/` and the parameter schema. Run
`make_talos_navigation` from `models/`, then rebuild with `model_manager.py` as above. Do not edit generated
code; change the model or the MATLAB functions and rebuild.

The estimator filters with the library Extended Kalman Filter block, and the MATLAB test checks the block
against a reference implementation.

## Tests

```bash
colcon test --packages-select riptide_navigation
```

or directly:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

- `test/test_config_schema.py` checks the schema, the checked-in config and the diagnostic key table.
- `matlab/test_talos_navigation_model.m` runs the estimator functions on synthetic sensor data (a DVL
  dropout and recovery, NaN samples, rest and motion) and checks that the model compiles and that the EKF
  block matches the reference filter. Run it from MATLAB R2023a with `matlab/` on the path.
