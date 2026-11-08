# Copyright 2026 OSU Underwater Robotics Team
# SPDX-License-Identifier: BSD-3-Clause
"""Load and validate the navigation config, and split it into model and overseer parameters.

`navigation_config.py json CONFIG` prints the model's values for MATLAB (talos_nav_params.m).
"""

import json
import math
import re
import sys
from pathlib import Path

import yaml

SCHEMA = Path(__file__).resolve().parent.parent / "config" / "parameter_schema.json"
ROOT = "navigation"


def load_schema(path=SCHEMA):
    return {entry["name"]: entry for entry in json.loads(Path(path).read_text())["parameters"]}


def full_name(entry):
    """Name the model reads: nesting joined with the double underscore the controller uses."""
    return "__".join((ROOT, entry["group"], entry["name"]))


def load_config(path):
    """Short-name values from a nested navigation config."""
    tree = yaml.safe_load(Path(path).read_text())
    values = {}
    for group, members in tree[ROOT].items():
        for name, value in members.items():
            values[name] = value
    return values


def split_type(kind):
    match = re.fullmatch(r"(\w+)(?:\[(\d+)\])?", kind)
    return match.group(1), (int(match.group(2)) if match.group(2) else None)


def scalar_errors(name, entry, kind, value):
    if kind == "bool":
        return [] if isinstance(value, bool) else [name + ": expected a bool"]
    if kind == "string":
        return [] if isinstance(value, str) else [name + ": expected a string"]
    if kind == "enum":
        return [] if value in entry["enum"] else [name + ": must be one of " + str(entry["enum"])]
    numeric = isinstance(value, (int, float)) and not isinstance(value, bool)
    if not numeric or (kind == "int" and not isinstance(value, int)):
        return [name + ": expected " + kind]
    if not math.isfinite(value):
        return [name + ": must be finite"]
    low = entry.get("min")
    if low is not None and (value < low or (entry.get("exclusive_min") and value == low)):
        bound = ">" if entry.get("exclusive_min") else ">="
        return [name + ": must be " + bound + " " + str(low)]
    if entry.get("max") is not None and value > entry["max"]:
        return [name + ": must be <= " + str(entry["max"])]
    return []


def validate(config, schema):
    errors = []
    for name in config:
        if name not in schema:
            errors.append(name + ": unknown parameter")
        elif schema[name].get("computed"):
            errors.append(name + ": is computed by the overseer, not set in the config")
    for name, entry in schema.items():
        if name not in config:
            continue
        value = config[name]
        kind, size = split_type(entry["type"])
        if size is None:
            errors += scalar_errors(name, entry, kind, value)
            continue
        if not isinstance(value, list) or len(value) != size:
            errors.append("%s: expected %d values" % (name, size))
            continue
        for index, item in enumerate(value):
            errors += scalar_errors("%s[%d]" % (name, index), entry, kind, item)
        for index, (item, allowed) in enumerate(zip(value, entry.get("supported", []))):
            if item is True and not allowed:
                errors.append("%s[%d]: enables an unsupported field" % (name, index))
    errors += cross_checks(config, schema)
    return errors


def cross_checks(config, schema):
    values = {name: config.get(name, entry["default"]) for name, entry in schema.items()}
    errors = []
    if values["max_queue_size"] > values["queue_capacity"]:
        errors.append("max_queue_size: exceeds the build-time queue_capacity")
    if values["frequency"] > values["estimator_rate"]:
        errors.append("frequency: exceeds the build-time estimator_rate")
    if values["base_rate"] % values["estimator_rate"] != 0:
        errors.append("estimator_rate: must divide the build-time base_rate")
    if values["estimator_rate"] % values["parameter_rate"] != 0:
        errors.append("parameter_rate: must divide the build-time estimator_rate")
    if values["history_length"] > values["max_measurement_lag"]:
        errors.append("history_length: exceeds max_measurement_lag")
    return errors


def ros_parameters(config, schema):
    """ROS parameters of the model node (full names) and of the overseer (short, native types)."""
    model, overseer = {}, {}
    for name, entry in schema.items():
        if entry.get("computed") or entry["consumer"] == "build":
            continue
        value = config.get(name, entry["default"])
        if entry["consumer"] in ("overseer", "both"):
            overseer[name] = value
        kind = split_type(entry["type"])[0]
        # strings are byte arrays, which a YAML file cannot express; the overseer sets them
        if entry["consumer"] in ("model", "both") and kind != "string":
            if kind == "enum":
                value = entry["enum"].index(value) + 1
            elif kind == "double" and isinstance(value, list):
                # R2023a Get Parameter reads double[] as int64[], so one double per element
                for index, item in enumerate(value):
                    model["%s_%d" % (full_name(entry), index)] = float(item)
                continue
            elif kind == "double":
                value = float(value)
            model[full_name(entry)] = value
    # the overseer reports with the same masks and modes the model runs
    for name in ("vectornav_imu_config", "fog_twist_config", "dvl_twist_config",
                 "depth_pose_config", "max_stamp_lead", "imu_acceleration_update",
                 "publication_mode"):
        overseer[name] = config.get(name, schema[name]["default"])
    return model, overseer


def model_values(config, schema):
    """Return the model's name -> value map: enums as 1-based indices, bools as 0/1."""
    tree = {}
    for name, entry in schema.items():
        if entry["consumer"] not in ("model", "both"):
            continue
        value = config.get(name, entry["default"])
        if entry["type"] == "enum":
            value = entry["enum"].index(value) + 1
        elif isinstance(value, bool):
            value = int(value)
        elif isinstance(value, list):
            value = [int(item) if isinstance(item, bool) else item for item in value]
        tree.setdefault(entry["group"], {})[name] = value
    return tree


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] != "json":
        sys.exit("usage: navigation_config.py json CONFIG")
    json.dump(model_values(load_config(sys.argv[2]), load_schema()), sys.stdout)
