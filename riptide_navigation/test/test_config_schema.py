# Copyright 2026 OSU Underwater Robotics Team
# SPDX-License-Identifier: BSD-3-Clause
import copy
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "launch"))
import navigation_config  # noqa: E402

SCHEMA = navigation_config.load_schema()
DEFAULTS = {name: entry["default"] for name, entry in SCHEMA.items() if not entry.get("computed")}


def test_defaults_validate():
    assert navigation_config.validate(DEFAULTS, SCHEMA) == []


def test_checked_in_config_matches_defaults():
    assert navigation_config.load_config(ROOT / "config" / "talos_navigation.yaml") == DEFAULTS


def test_rejects_bad_values():
    bad = copy.deepcopy(DEFAULTS)
    bad["dvl_twist_config"][0] = True
    bad["process_noise_diag"] = [0.0] * 15
    bad["publication_mode"] = "fast"
    bad["dvl_covariance_multiplier"] = 0.0
    bad["max_queue_size"] = 4096
    bad["unknown"] = 1
    bad["imu_translation"] = [0.0, 0.0, 0.0]
    errors = "\n".join(navigation_config.validate(bad, SCHEMA))
    for expected in ("dvl_twist_config[0]", "process_noise_diag", "publication_mode",
                     "dvl_covariance_multiplier", "max_queue_size", "unknown",
                     "imu_translation: is computed"):
        assert expected in errors


def test_names_are_unique_and_typed():
    names = [entry["name"] for entry in SCHEMA.values()]
    assert len(names) == len(set(names))
    for entry in SCHEMA.values():
        assert entry["consumer"] in ("model", "overseer", "build", "both")


def test_full_names_fit_a_matlab_variable():
    for entry in SCHEMA.values():
        assert len(navigation_config.full_name(entry)) <= 63


def test_model_sizes_match_build_group():
    text = (ROOT / "matlab" / "talos_nav_sizes.m").read_text()
    sizes = dict(re.findall(r"s\.(\w+) = (\d+);", text))
    assert int(sizes["queue"]) == DEFAULTS["queue_capacity"]
    assert int(sizes["inputs"]) == DEFAULTS["input_history_capacity"]
    assert int(sizes["checkpoints"]) == DEFAULTS["checkpoint_capacity"]
    assert int(sizes["ring"]) == DEFAULTS["input_ring_capacity"]


def test_diagnostic_keys_match_the_model():
    keys = json.loads((ROOT / "config" / "diagnostic_keys.json").read_text())
    assert len(keys) == len(set(keys))
    assert "d(%d) =" % len(keys) in (ROOT / "matlab" / "talos_nav_diag.m").read_text()


def test_ros_parameters_split():
    model, overseer = navigation_config.ros_parameters(DEFAULTS, SCHEMA)
    assert model["navigation__timing__publication_mode"] == 2
    assert "navigation__frames__world_frame" not in model
    assert isinstance(model["navigation__timing__frequency"], float)
    assert model["navigation__timing__sensor_timeouts_3"] == 0.5
    assert "navigation__timing__sensor_timeouts" not in model
    assert max(len(name) for name in model) <= 63
    assert not any("mounts" in name for name in model)
    assert overseer["world_frame"] == "odom"
    assert overseer["publication_mode"] == "queued_updates"
