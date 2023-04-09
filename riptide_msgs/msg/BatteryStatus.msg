# general cell info
string cell_name
uint16 serial
uint8 detect

# runtime info
float32 pack_voltage
float32 pack_current
float32 average_current
uint16 time_to_dischg
uint8 soc

# constants
uint8 DETECT_NONE = 0
uint8 DETECT_STBD = 2
uint8 DETECT_PORT = 1