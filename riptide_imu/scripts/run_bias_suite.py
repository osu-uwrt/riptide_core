#!/usr/bin/env python3
"""Guide an operator through the full VN-100 bias acceptance suite."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def pause(instruction):
    """Print one operator instruction and wait for acknowledgement."""
    print(f"\n{instruction}")
    input("Press Enter when complete, or Ctrl-C to abort: ")


def finite_vector(value, length=3):
    """Return whether a JSON value is a finite numeric vector."""
    return (isinstance(value, list) and len(value) == length and
            all(isinstance(item, (int, float)) and math.isfinite(item)
                for item in value))


def assess(report):
    """Evaluate the exported application report and return named checks."""
    checks = {}
    for name in ("gyro", "accelerometer", "magnetometer", "gravity_report"):
        checks[f"{name}_quality"] = bool(report.get(name, {}).get("passed"))
    checks["gyro_vector_finite"] = finite_vector(
        report.get("gyro", {}).get("residual_bias_rad_s"))
    checks["accelerometer_vector_finite"] = finite_vector(
        report.get("accelerometer", {}).get("bias_m_s2"))
    checks["magnetometer_vector_finite"] = finite_vector(
        report.get("magnetometer", {}).get("in_plane_bias_gauss"))
    gravity = report.get("gravity_report", {})
    checks["gravity_vector_finite"] = finite_vector(
        gravity.get("gravity_vector_m_s2"))
    checks["gravity_magnitude_physical"] = (
        isinstance(gravity.get("magnitude_m_s2"), (int, float)) and
        7.0 < gravity["magnitude_m_s2"] < 12.0)
    checks["magnetic_coverage"] = (
        report.get("magnetometer", {}).get("coverage_deg", 0.0) >=
        report.get("policy", {}).get("magnetometer_min_coverage_deg", 300.0))
    checks["sign_and_frame_documented"] = (
        report.get("gravity_sign") ==
        "gravity_vector_sensor = -mean_stationary_specific_force" and
        bool(report.get("frame")))
    return checks


def main():
    """Run the guided suite and return a test-style process status."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True,
                        help="Path to select in the dashboard Export Report dialog")
    parser.add_argument("--result", type=Path,
                        help="Suite result JSON; defaults beside --report")
    parser.add_argument("--remote-namespace", default="",
                        help="Namespace of the running riptide_imu driver")
    parser.add_argument("--no-launch", action="store_true",
                        help="Use an already-running dashboard")
    args = parser.parse_args()
    result_path = args.result or args.report.with_name(
        args.report.stem + "_suite_result.json")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    process = None
    try:
        if not args.no_launch:
            command = ["ros2", "launch", "riptide_imu",
                       "sensor_config.launch.py", "connection_mode:=ros",
                       f"remote_namespace:={args.remote_namespace}"]
            process = subprocess.Popen(command)
            time.sleep(3.0)
        print("VN-100 full bias suite. This run captures and evaluates data only.")
        print("Do not Apply or Save Persistent Settings during this acceptance run.")
        pause("PREPARATION: Confirm ROS mode is connected, live telemetry is fresh, "
              "the robot is secured, thrusters are disabled, and the configuration backup exists.")
        pause("GYRO: Select Calibration > Gyroscope. With the robot completely stationary, "
              "capture at least 10 seconds. Stop capture before touching the robot. Repeat at "
              "two or more arbitrary orientations, then Evaluate. Do not assume any pose is "
              "level.")
        pause("ACCELEROMETER: Advance to Accelerometer. Capture stationary windows at varied "
              "reachable tilts spanning both signs of roll and pitch (target 8 or more poses). "
              "Stop capture while repositioning, then Evaluate.")
        pause("MAGNETOMETER: Advance to Magnetometer. Keep tilt variation small while rotating "
              "the whole robot through a slow full horizontal circle, away from movable ferrous "
              "objects and high-current wiring. Evaluate the limited planar fit.")
        pause("GRAVITY/TILT: Advance to Gravity Report. Hold the robot stationary in its normal "
              "installed pose for at least 10 seconds, then Evaluate.")
        pause(f"REVIEW: Inspect every quality message. Click Export Report and save exactly to:\n"
              f"  {args.report}")
        if not args.report.is_file():
            raise RuntimeError(f"Calibration report was not found: {args.report}")
        report = json.loads(args.report.read_text(encoding="utf-8"))
        checks = assess(report)
        passed = all(checks.values())
        result = {
            "schema_version": 1,
            "passed": passed,
            "checks": checks,
            "calibration_report": str(args.report.resolve()),
            "coefficients_applied": False,
            "persistent_save_performed": False,
            "note": "Gravity tilt has no yaw and is not a sensor-to-robot mounting rotation.",
        }
        result_path.write_text(json.dumps(result, indent=2) + "\n",
                               encoding="utf-8")
        for name, outcome in checks.items():
            print(f"{'PASS' if outcome else 'FAIL'}  {name}")
        print(f"\nOVERALL: {'PASS' if passed else 'FAIL'}")
        print(f"Result: {result_path}")
        return 0 if passed else 1
    except (KeyboardInterrupt, EOFError):
        print("\nABORTED: no calibration coefficients were applied by this script.")
        return 2
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"\nFAIL: {error}", file=sys.stderr)
        return 1
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__ == "__main__":
    raise SystemExit(main())
