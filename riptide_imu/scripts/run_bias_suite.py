#!/usr/bin/env python3
"""Guide an operator through the full VN-100 bias acceptance suite."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time


SERVICE_SUFFIX = "/vectornav/config"


def normalize_namespace(value):
    """Normalize a ROS namespace, using slash for the root namespace."""
    value = value.strip()
    if value.endswith(SERVICE_SUFFIX):
        value = value[:-len(SERVICE_SUFFIX)]
    value = "/" + value.strip("/") if value.strip("/") else "/"
    return value


def discover_endpoints():
    """Return namespaces exposing the VectorNav configuration service."""
    completed = subprocess.run(
        ["ros2", "service", "list", "-t"], capture_output=True, text=True,
        timeout=5, check=False)
    if completed.returncode:
        raise RuntimeError("Could not inspect ROS services: " +
                           (completed.stderr.strip() or "ros2 failed"))
    endpoints = []
    for line in completed.stdout.splitlines():
        service = line.split()[0] if line.split() else ""
        if service.endswith(SERVICE_SUFFIX) and "QueryImuSerial" in line:
            endpoints.append(normalize_namespace(service))
    return sorted(set(endpoints))


def resolve_endpoint(requested, discovered):
    """Resolve and validate the requested driver namespace."""
    if requested:
        endpoint = normalize_namespace(requested)
        if endpoint not in discovered:
            available = ", ".join(discovered) if discovered else "none"
            raise RuntimeError(
                f"Requested ROS driver namespace {endpoint} is unavailable; "
                f"discovered endpoints: {available}")
        return endpoint
    if len(discovered) == 1:
        return discovered[0]
    if not discovered:
        raise RuntimeError(
            "No QueryImuSerial service ending in /vectornav/config was found; "
            "start the VN driver before the bias suite")
    raise RuntimeError(
        "Multiple VectorNav drivers were discovered; rerun with "
        "--remote-namespace set to one of: " + ", ".join(discovered))


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
    minimum_samples = report.get("policy", {}).get("minimum_samples", 200)
    for name in ("gyro", "accelerometer", "magnetometer", "gravity_report"):
        section = report.get(name, {})
        checks[f"{name}_evaluated"] = bool(
            section.get("evaluated", section.get("quality")))
        checks[f"{name}_samples"] = (
            section.get("sample_count", 0) >= minimum_samples)
        checks[f"{name}_quality"] = bool(section.get("passed"))
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
            endpoint = resolve_endpoint(args.remote_namespace,
                                        discover_endpoints())
            print(f"Using ROS VectorNav driver endpoint: {endpoint}")
            command = ["ros2", "launch", "riptide_imu",
                       "sensor_config.launch.py", "connection_mode:=ros",
                       f"remote_namespace:={endpoint}"]
            process = subprocess.Popen(command)
            time.sleep(3.0)
        print("VN-100 full bias suite. This run captures and evaluates data only.")
        print("Do not Apply or Save Persistent Settings during this acceptance run.")
        pause("PREPARATION: Confirm ROS mode says connected, live telemetry is "
              "fresh, the robot is secured, thrusters are disabled, and the "
              "configuration backup exists. Leave the sensor powered until "
              "its displayed temperature has stabilized.")
        pause("GYRO: In the Calibration dock select 'Gyro bias'. Click Start "
              "capture. With the robot completely stationary, capture at "
              "least 10 seconds. Stop capture before touching the robot. "
              "Repeat at two or more safe 3D orientations with both roll and "
              "pitch variation; yaw-only movement is insufficient. Wait for "
              "the robot to settle before restarting capture. Confirm the "
              "displayed count exceeds the minimum, click Evaluate quality, "
              "and verify a non-empty quality message. No pose must be level.")
        pause("ACCELEROMETER: Select 'Accelerometer bias + magnitude'. "
              "Capture stationary windows at varied reachable tilts spanning "
              "both signs of roll and pitch (target 8 or more poses). Exact "
              "angles and a level pose are unnecessary. Stop capture while "
              "repositioning and wait for settling. Confirm the minimum count, click "
              "Evaluate quality, and verify the quality message.")
        pause("MAGNETOMETER: Select 'Planar magnetometer'. Keep tilt variation "
              "small and constant while rotating the whole robot through a slow full "
              "horizontal circle, away from movable ferrous objects and "
              "high-current wiring. Confirm the minimum count, click Evaluate "
              "quality, and inspect the limited planar fit.")
        pause("GRAVITY/TILT: Select 'Gravity/tilt report'. Hold the robot "
              "stationary in its normal installed pose for at least 10 "
              "seconds. Confirm the minimum count, click Evaluate quality, "
              "and verify the quality message.")
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
        for name in ("gyro", "accelerometer", "magnetometer", "gravity_report"):
            section = report.get(name, {})
            if not section.get("passed"):
                print(f"  {name}: samples={section.get('sample_count', 'not recorded')}; "
                      f"{section.get('quality') or 'not evaluated'}")
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
