# Note that these values are not gaurenteed to be sampled at the same time internally. Cases can occur where
# thrusters_moving is true, disabled_flags will be non-zero. When using telemetry, ensure that you do not depend that
# there be coherency across values. This was done to simplify firmware, so it doesn't need lock out the entire DShot
# control loop + interrupt system to sample telemetry.

# Set to true if the ESC board considers that the board is receiving enough power for ESCs to start up
bool escs_powered
# Set to true if the DShot controller is commanding a non-zero value to any thruster
bool thrusters_moving
# Contains the first thruster index for this packet. Used to distinguish across ESC boards
uint8 start_thruster_num
# Bitwise array of reasons that dshot is disabled. Can be useful when diagnosing why thrusters are not moving
uint8 disabled_flags
# The measured voltage of the ESC board V+ rail from the RP2040 analog pin. Used when determining if the ESC board is on
float32 vcc_voltage
# The time delta in microseconds since the last telemetry packet was sent. Useful for computing controller rates
uint32 packet_delta_us
# The total number of controller ticks since the last telemetry packet was sent
uint16 tick_cnt
# Array of individual ESC's telemetry data
riptide_msgs2/DshotSingleTelemetry[4] esc_telemetry
