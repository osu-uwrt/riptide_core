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
        "gyro": {"passed": True, "residual_bias_rad_s": [0, 0, 0]},
        "accelerometer": {"passed": True, "bias_m_s2": [0, 0, 0]},
        "magnetometer": {
            "passed": True,
            "in_plane_bias_gauss": [0, 0, 0],
            "coverage_deg": 310,
        },
        "gravity_report": {
            "passed": True,
            "gravity_vector_m_s2": [0, 0, -9.8],
            "magnitude_m_s2": 9.8,
        },
        "policy": {"magnetometer_min_coverage_deg": 300},
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


if __name__ == "__main__":
    unittest.main()
