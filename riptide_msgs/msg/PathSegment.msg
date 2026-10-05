# How a FollowPath goal reaches one of its path points from the point before it
# (the first from where the vehicle is). Points are in the path point's header frame.

uint8 LINE=0
uint8 ARC=1
uint8 shape

# ARC: turns `sweep` radians about `center` (+ = counterclockwise seen from above,
# about the frame's +z; more than 2*pi goes round more than once). Only the
# horizontal part of center is used. The path point sets where the arc ends:
# its distance from center (the radius changes linearly with the angle, so a
# different end radius makes a spiral) and its depth (a helix); the arc ends at
# that radius and depth, `sweep` from where it started.
geometry_msgs/Point center
float64 sweep

# Heading (yaw) while following this segment; roll and pitch always blend from
# the previous point's orientation to this one's.
uint8 HEADING_WAYPOINT=0 # blend the yaw from the previous point's to this one's
uint8 HEADING_PATH=1     # face the direction of travel
uint8 HEADING_LOOK_AT=2  # face look_at
uint8 heading
geometry_msgs/Point look_at
# HEADING_LOOK_AT: if set, look_at is in this TF frame instead and follows it
# live while on this segment (the frame may move; the path itself does not).
string look_at_frame
float64 yaw_offset # added to the HEADING_PATH / HEADING_LOOK_AT yaw; pi/2 travels sideways
float64 spin       # extra yaw over the segment (+ = counterclockwise), spread by progress; 2*pi = one turn
# Steady spin: extra yaw per metre travelled (+ = counterclockwise), carried on
# across segments as one continuous spin; the path's total is rounded to whole
# turns so it still ends on its planned heading. About rad/s / cruise speed.
float64 spin_rate
