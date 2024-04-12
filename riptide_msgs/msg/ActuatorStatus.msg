uint8 CLAW_ERROR=0
uint8 CLAW_DISARMED=1
uint8 CLAW_UNKNOWN=2
uint8 CLAW_OPENED=3
uint8 CLAW_CLOSED=4
uint8 CLAW_OPENING=5
uint8 CLAW_CLOSING=6

uint8 TORPEDO_ERROR=0
uint8 TORPEDO_DISARMED=1
uint8 TORPEDO_CHARGING=2
uint8 TORPEDO_CHARGED=3
uint8 TORPEDO_FIRING=4
uint8 TORPEDO_FIRED=5

uint8 DROPPER_ERROR=0
uint8 DROPPER_DISARMED=1
uint8 DROPPER_READY=2
uint8 DROPPER_DROPPING=3
uint8 DROPPER_DROPPED=4

bool actuators_armed
uint8 claw_state
uint8 torpedo_state
uint8 torpedo_available_count
uint8 dropper_state
uint8 dropper_available_count