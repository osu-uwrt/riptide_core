#!/usr/bin/env python3
"""Tests for the guided bias-suite report checks."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts/run_bias_suite.py"
SPEC = importlib.util.spec_from_file_location("run_bias_suite", SCRIPT)
BIAS_SUITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BIAS_SUITE)


def passing_report():
    """Return a minimally complete passing exported report."""
    return {
        "gyro": {"evaluated": True, "sample_count": 200, "passed": True,
                 "quality": "Passed", "residual_bias_rad_s": [0, 0, 0]},
        "accelerometer": {"evaluated": True, "sample_count": 200,
                          "passed": True, "quality": "Passed",
                          "bias_m_s2": [0, 0, 0]},
        "magnetometer": {
            "evaluated": True,
            "sample_count": 200,
            "passed": True,
            "quality": "Passed",
            "in_plane_bias_gauss": [0, 0, 0],
            "coverage_deg": 310,
        },
        "gravity_report": {
            "evaluated": True,
            "sample_count": 200,
            "passed": True,
            "quality": "Passed",
            "gravity_vector_m_s2": [0, 0, -9.8],
            "magnitude_m_s2": 9.8,
        },
        "policy": {"minimum_samples": 200,
                   "magnetometer_min_coverage_deg": 300},
        "gravity_sign": (
            "gravity_vector_sensor = -mean_stationary_specific_force"),
        "frame": "VectorNav configured sensor output frame",
    }


class BiasSuiteAssessmentTest(unittest.TestCase):
    """Validate pass/fail aggregation and required report evidence."""

    def test_complete_quality_report_passes(self):
        """All application gates and report metadata are required."""
        self.assertTrue(all(BIAS_SUITE.assess(passing_report()).values()))

    def test_failed_math_gate_fails_suite(self):
        """A single failed calibration step fails the full suite."""
        report = passing_report()
        report["gyro"]["passed"] = False
        self.assertFalse(all(BIAS_SUITE.assess(report).values()))

    def test_missing_capture_evidence_fails_suite(self):
        """Default vectors cannot disguise a capture that never happened."""
        report = passing_report()
        report["gyro"].update(evaluated=False, sample_count=0, quality="")
        checks = BIAS_SUITE.assess(report)
        self.assertFalse(checks["gyro_evaluated"])
        self.assertFalse(checks["gyro_samples"])

    def test_endpoint_resolution_rejects_wrong_namespace(self):
        """A requested namespace must match a discovered driver."""
        with self.assertRaisesRegex(RuntimeError, "discovered endpoints: /"):
            BIAS_SUITE.resolve_endpoint("/talos", ["/"])
        self.assertEqual(BIAS_SUITE.resolve_endpoint("", ["/"]), "/")


if __name__ == "__main__":
    unittest.main()
