# Set to true if the ESC is communicating over UART telemetry
bool esc_online
# Set to true if the thruster is ready and able to generate thrust, false if an error is detected
# It is possible for an ESC to be online, but the thruster to be unplugged or damaged. If no movement is detected, the
# thruster will be disabled until the ESC is power cycled, and all dshot output will be disabled on this thruster
bool thruster_ready
# Temperature of ESC in Deg C from DShot UART Telemetry
uint8 temperature_c
# Signed RPM of the ESC from DShot UART Telemetry
int16 rpm
# Voltage in Volts of the ESC V+ rail from DShot UART Telemetry
float32 voltage
# Current flow in Amps through the ESC from DShot UART Telemetry
float32 current
# Current Consumption in Amp Hours of the ESC from DShot UART Telemetry
float32 consumption_ah
# Count of missed controller ticks (but still sending old commands) since last telemetry packet
# A non-zero value is acceptable for this field, but means that this thruster's controller skipped control loop ticks
uint16 dshot_missed_cnt
# Count of controller ticks where the ESC was disabled due to no dshot RPM feedback
# A non-zero value in this field means that the controller operation is impacted, as it has killed that ESC's controller
uint16 dshot_offline_cnt
# Count of total uart telemtry packets received sent since last telemetry packet
uint16 uart_success_cnt
# Count of missed/corrupted uart telemetry responses (but not enough to mark ESC as not present) since last telemetry packet
# A non-zero value in this field is okay, but that means that uart telemetry may be a bit stale
uint16 uart_missed_cnt
# Count of uart telemetry requests that had the ESC considered not present since last telemetry packet
# A non-zero value in this field means that there are periods when this telemetry packet would report not present
uint16 uart_disabled_cnt
