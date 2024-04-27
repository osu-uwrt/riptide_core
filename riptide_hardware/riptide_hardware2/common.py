import enum
import rclpy.clock

def get_time_latency(clock, start_time):
    now = clock.now().to_msg()
    return (now.sec - start_time.sec) + ((now.nanosec - start_time.nanosec) / 1000000000.0)

class ExpiringMessage:
    def __init__(self, clock: rclpy.clock.Clock, message_life: float, has_header=False):
        self._clock = clock
        self._has_header = has_header

        self._value = None
        self._receive_time = None
        self._message_life = message_life

    def update_value(self, value):
        self._value = value
        if self._has_header:
            self._receive_time = value.header.stamp
        else:
            self._receive_time = self._clock.now().to_msg()

    def get_value(self):
        if self._receive_time is None:
            return None

        latency = get_time_latency(self._clock, self._receive_time)
        if latency < self._message_life:
            return self._value
        else:
            return None

    def force_expire(self):
        self._receive_time = None

# Constants from Firmware
class Mk2Board(enum.Enum):
    def __new__(cls, bus_id: int, client_id: int, board_name: str, friendly_name: str) -> 'Mk2Board':
        obj = object.__new__(cls)
        obj._value_ = friendly_name + "-" + str(client_id)
        obj._board_name_ = board_name
        obj._bus_id_ = bus_id
        obj._client_id_ = client_id
        obj._friendly_name_ = friendly_name
        return obj

    @property
    def client_id(self) -> int:
        return self._client_id_

    @property
    def bus_id(self) -> int:
        return self._bus_id_

    @property
    def board_name(self) -> str:
        return self._board_name_

    @property
    def friendly_name(self) -> str:
        return self._friendly_name_

    POWER_BOARD = 1, 1, "mk2_power_board", "Power Board"
    ESC_BOARD_0 = 1, 2, "mk2_esc_board", "ESC Board 0"
    ESC_BOARD_1 = 1, 3, "mk2_esc_board", "ESC Board 1"
    CAMERA_CAGE_BB = 1, 4, "mk2_camera_cage_bb", "Camera Cage BB"
    ACTUATOR_BOARD = 1, 5, "mk2_actuator_board", "Actuator Board"
    SBH_MCU_PORT = 2, 1, "sbh_mcu", "Smart Battery Housing Port"  # Note the bus_id actually corresponds to detect id
    SBH_MCU_STBD = 2, 2, "sbh_mcu", "Smart Battery Housing Stbd"

    PUDDLES_BACKPLANE = 0, 1, "puddles_backplane", "Puddles Backplane"
