# Current task

Goal: VectorNav Workshop (sensor_config) usability round requested 2026-10-21, then hardware validation.

Checklist (`[x]` built at the workspace root and covered by tests; 168 tests, 0 failures, 105 cppcheck skips):

- [x] 1. Attitude-over-time graph: Signals › Attitude (roll, pitch, unwrapped yaw), 20 s to 30 min spans
- [x] 2. Drift indicator in Live data: rate over the last 60 s still, total since restart
- [x] 3. 20 Hz explained (local serial polls every 50 ms); sensor output rate shown from registers; config
      checked against the pre-test backup (output unchanged; register 21 differs, see below)
- [x] 4. Auto-read on every register click while connected, re-clicks included (not staged ones, not 80)
- [x] 5. Line-load meter on the command bar while connected; Apply and Save to flash refuse >100%
- [x] 6. Manual register and field names (`config/registers.json`, schema 2)
- [x] 7. Binary outputs as port / rate / field checkboxes; enum fields as menus, unknown values kept
- [x] 8. Plot axis toggles; scale fits the shown axes
- [x] 9. American spelling; sensor model and scene scale in millimeters
- [x] 10. Robot config (talos.yaml / mercury_xacro_frames.yaml) for directions; remembered profiles
- [x] 11. Temperature: Steady / Changing with rising / falling, rate and 15 s hysteresis, fixed layout
- [x] 12. Checks separated from the Result card (value, 3σ, before / after register bias); scrolling columns
- [x] 13. Magnetometer coverage starts at 0 (span-based, still-gyro bias removed)
- [x] 14. Gravity report roll / pitch; with a robot config, robot roll / pitch and corrected `pose:` line
- [x] 15. Readback compares at the sensor's float32 / six-digit precision (tool and driver)
- [x] 16. Export protocol log (timestamped)
- [x] 17. Staged / applied state read from the registers; "Stage latest results" re-stages after discard
- [x] 18. Orbiting switches the view preset to 3D; presets act on every click
- [x] 19. Port / namespace / baud boxes open their list on click and filter as you type; the port /
      namespace box rescans on every click without connecting
- [x] 20. Level-robot disclaimer shown with the robot gravity frame and kept in the copied pose line

Package layout (2026-10-21): the only executables are `imu` and `sensor_config`; the tool is built only
when Qt5, Eigen and yaml-cpp are found. Its fonts, icons, `registers.json` and `calibration_policy.json` are
in `config_tool_assets/`, and `config/` holds only the driver's `vectornav.yaml`. The bias-suite script,
`hardware_acceptance.py` and `vn_calibration_sim` were removed; the calibration simulator lives on as
test-only code in `test/config_tool/`. vnproglib stays bundled in the package for now.

Open questions for the user:
- Register 21 (magnetic and gravity reference) read `-1.77034;1.35946;0;0;0;-9.79375` on the sensor but
  `0.234;0;0.4212;0;0;-9.79375` in `hardware_backups/…pretest_active_2026-10-16.json`. Nothing here writes
  it. Not restored without the user's go-ahead; Import XML of the pre-test XML would stage it.
- The robot's driver must be rebuilt and restarted for the ROS-mode readback fix.

Earlier state (unchanged): hardware register reads, live telemetry, same-value writes, ROS maintenance gating,
flash save/reset and a 460800/230400 baud round trip passed on VN-100T-CR 0100059698 on 2026-10-16. The
wizard has not run against the sensor yet. Backups in `hardware_backups/` stay uncommitted.

Next: run each wizard step on the robot over ROS and over local serial; check the line-load figure against
the driver's packet rate; Windows Control Center bidirectional XML check.
