#!/usr/bin/env python3
"""Non-destructive VN-100 acceptance checks with an opt-in flash cycle."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import select
import termios
import time
import uuid


BAUDS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


def checksum(payload):
    value = 0
    for byte in payload.encode("ascii"):
        value ^= byte
    return f"{value:02X}"


class Sensor:
    """Exclusive ASCII protocol connection used only by hardware acceptance."""

    def __init__(self, port, baud):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = termios.IGNPAR
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = BAUDS[baud]
        attrs[5] = BAUDS[baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buffer = b""

    def close(self):
        os.close(self.fd)

    def command(self, payload, expected, timeout=1.5):
        wire = f"${payload}*{checksum(payload)}\r\n".encode("ascii")
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                os.write(self.fd, wire)
                next_send = now + 0.2
            readable, _, _ = select.select([self.fd], [], [], 0.1)
            if not readable:
                continue
            self.buffer += os.read(self.fd, 4096)
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if self._matches(text, expected):
                    return text
                if text.startswith("$VNERR"):
                    raise RuntimeError(text)
        raise TimeoutError(f"No {expected} response to {payload}")

    @staticmethod
    def _matches(response, expected):
        """Match register replies even when firmware zero-pads the ID."""
        if response.startswith(f"${expected}"):
            return True
        expected_fields = expected.split(",")
        response_fields = response.removeprefix("$").split("*")[0].split(",")
        return (len(expected_fields) == 2 and len(response_fields) >= 2 and
                expected_fields[0] == "VNRRG" and
                response_fields[0] == "VNRRG" and
                int(expected_fields[1]) == int(response_fields[1]))

    def read(self, register_id):
        response = self.command(f"VNRRG,{register_id}", f"VNRRG,{register_id}")
        body = response[1:response.index("*")]
        return body.split(",")[2:]

    def write(self, register_id, values):
        self.command(f"VNWRG,{register_id}," + ",".join(values),
                     f"VNWRG,{register_id}")
        actual = self.read(register_id)
        if actual != values:
            raise AssertionError(
                f"Register {register_id} readback differs: {actual} != {values}")

    def save(self):
        self.command("VNWNV", "VNWNV", timeout=3.0)

    def reset(self):
        self.command("VNRST", "VNRST")
        time.sleep(2.0)
        termios.tcflush(self.fd, termios.TCIOFLUSH)


class RosSensor:
    """Configuration access through the running robot driver."""

    def __init__(self, namespace):
        import rclpy
        from riptide_msgs2.srv import QueryImuSerial

        self.rclpy = rclpy
        self.query_type = QueryImuSerial
        rclpy.init(args=None)
        self.node = rclpy.create_node(f"vn_acceptance_{uuid.uuid4().hex[:8]}")
        prefix = namespace.strip("/")
        service = f"/{prefix}/vectornav/config" if prefix else "/vectornav/config"
        self.client = self.node.create_client(QueryImuSerial, service)
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Configuration service unavailable: {service}")
        self.session = ""
        self.pending_expected = {}
        self.save_already_reset = False

    def request(self, operation, payload=None, maintenance=False, timeout=15.0):
        if maintenance and not self.session:
            opened = self.request("open", {"client_id": self.node.get_name()})
            self.session = opened["data"]["session"]
        document = {"version": 1, "operation": operation,
                    "payload": payload or {}}
        if self.session:
            document["session"] = self.session
        if operation in {"write", "reset", "save_reset", "tare"}:
            document["operation_id"] = str(uuid.uuid4())
        request = self.query_type.Request()
        request.request = json.dumps(document, separators=(",", ":"))
        future = self.client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future,
                                              timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"ROS operation timed out: {operation}")
        response = json.loads(future.result().response)
        if not response.get("ok"):
            raise RuntimeError(f"{response.get('code')}: {response.get('message')}")
        return response

    def read(self, register_id):
        response = self.request("read", {"register": register_id})
        return [str(value) for value in response["data"]["values"]]

    def write(self, register_id, values):
        payload = {"register": register_id, "values": values}
        response = self.request("write", payload,
                                maintenance=register_id in
                                {5, 6, 7, 26, 30, 32, 75, 76, 77, 82})
        actual = [str(value) for value in response["data"]["values"]]
        if actual != values:
            raise AssertionError(
                f"Register {register_id} readback differs: {actual} != {values}")
        self.pending_expected[str(register_id)] = values

    def save(self):
        self.request("save_reset", {"expected": self.pending_expected},
                     maintenance=True, timeout=20.0)
        self.pending_expected = {}
        self.save_already_reset = True

    def reset(self):
        if self.save_already_reset:
            self.save_already_reset = False
            return
        self.request("reset", maintenance=True, timeout=15.0)

    def close(self):
        try:
            if self.session:
                self.request("close")
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()


def load_schema(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)["registers"]


def snapshot(sensor, schema):
    result = {}
    failures = {}
    for definition in schema:
        register_id = definition["id"]
        if register_id == 80:
            failures[str(register_id)] = "Skipped: reading resets integration interval"
            continue
        try:
            result[str(register_id)] = sensor.read(register_id)
        except (RuntimeError, TimeoutError) as error:
            failures[str(register_id)] = str(error)
    return result, failures


def write_journal(path, phase, active, saved=None, detail=None):
    document = {
        "schema_version": 1,
        "phase": phase,
        "active_snapshot": active,
        "saved_snapshot": saved,
        "detail": detail,
        "warning": "If incomplete, restore registers from this journal before reuse.",
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--imu-port")
    connection.add_argument("--remote-namespace",
                            help="Use the running driver in this ROS namespace")
    parser.add_argument("--baud", type=int, default=460800, choices=BAUDS)
    parser.add_argument("--schema", type=Path)
    parser.add_argument("--journal", type=Path,
                        default=Path("vn_hardware_recovery.json"))
    parser.add_argument("--snapshot-only", action="store_true",
                        help="Read active configuration and exit without writes")
    parser.add_argument("--flash-persistence", action="store_true",
                        help="Opt in to reset/save cycles and temporary tag change")
    args = parser.parse_args()
    if args.schema is None:
        source_schema = Path(__file__).parents[1] / "config/registers.json"
        installed_schema = (Path(__file__).resolve().parent / "../../share/riptide_imu/"
                            "config/registers.json").resolve()
        args.schema = source_schema if source_schema.exists() else installed_schema
    schema = load_schema(args.schema)
    sensor = (RosSensor(args.remote_namespace) if args.remote_namespace is not None
              else Sensor(args.imu_port, args.baud))
    try:
        active, failures = snapshot(sensor, schema)
        required = {"1", "2", "3", "4", "23", "25", "26", "43", "84"}
        missing = required - active.keys()
        if missing:
            raise RuntimeError(f"Required recovery snapshot incomplete: {sorted(missing)}")
        write_journal(args.journal, "active_snapshot_complete", active,
                      detail={"unsupported_or_skipped": failures})

        if args.snapshot_only:
            print(json.dumps({"passed": True, "snapshot_only": True,
                              "registers_read": len(active),
                              "journal": str(args.journal)}, indent=2))
            return

        # Exercise supported writes without changing values or flash.
        for register_id in (23, 25, 43, 84):
            sensor.write(register_id, active[str(register_id)])
        write_journal(args.journal, "volatile_same-value_writes_verified", active,
                      detail={"unsupported_or_skipped": failures})

        if args.flash_persistence:
            sensor.reset()
            saved, saved_failures = snapshot(sensor, schema)
            if required - saved.keys():
                raise RuntimeError("Saved-state snapshot incomplete; flash test aborted")
            writable_ids = {str(item["id"]) for item in schema
                            if item.get("writable") and
                            item.get("persistent", True)}
            active_differences = {key for key in writable_ids
                                  if key in active and key in saved
                                  and active[key] != saved[key]}
            if active_differences:
                raise RuntimeError(
                    "Active state differs from saved state; flash test aborted "
                    f"before mutation because exact restoration could interrupt "
                    f"communication: {sorted(active_differences)}")
            write_journal(args.journal, "saved_snapshot_complete", active, saved,
                          {"saved_failures": saved_failures})

            original_tag = saved.get("0", [""])
            test_tag = ["VN_ACCEPTANCE_TEST"]
            sensor.write(0, test_tag)
            sensor.save()
            sensor.reset()
            if sensor.read(0) != test_tag:
                raise AssertionError("Temporary tag did not persist across reset")
            write_journal(args.journal, "temporary_value_persisted", active, saved)

            sensor.write(0, original_tag)
            sensor.save()
            sensor.reset()
            if sensor.read(0) != original_tag:
                raise AssertionError("Original saved tag was not restored")
            restored, restored_failures = snapshot(sensor, schema)
            mismatches = {key for key in writable_ids
                          if key in saved and key in restored
                          and saved[key] != restored[key]}
            if mismatches:
                raise AssertionError(
                    f"Saved configuration restoration mismatch: {sorted(mismatches)}")
            write_journal(args.journal, "complete", active, saved,
                          {"message": "Original saved state restored and verified",
                           "restored_failures": restored_failures})
        else:
            write_journal(args.journal, "complete", active, detail={
                "flash_persistence": "not requested",
                "unsupported_or_skipped": failures,
            })
        print(json.dumps({"passed": True, "registers_read": len(active),
                          "journal": str(args.journal)}, indent=2))
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
