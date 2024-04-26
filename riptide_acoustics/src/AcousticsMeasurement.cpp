#include "riptide_acoustics/AcousticsMeasurement.h"

AcousticsMeasurement::AcousticsMeasurement(geometry_msgs::msg::Transform port, geometry_msgs::msg::Transform starboard, double Delta_T, double Timestamp){
    //load in states of pods when measurement was taken
    port_position.push_back(port.translation.x);
    port_position.push_back(port.translation.y);
    port_position.push_back(port.translation.z);

    port_orientation.push_back(port.rotation.x);
    port_orientation.push_back(port.rotation.y);
    port_orientation.push_back(port.rotation.z);
    port_orientation.push_back(port.rotation.w);

    starboard_position.push_back(starboard.translation.x);
    starboard_position.push_back(starboard.translation.y);
    starboard_position.push_back(starboard.translation.z);

    starboard_orientation.push_back(starboard.rotation.x);
    starboard_orientation.push_back(starboard.rotation.y);
    starboard_orientation.push_back(starboard.rotation.z);
    starboard_orientation.push_back(starboard.rotation.w);

    //fill in times
    delta_t = Delta_T;
    timestamp = Timestamp;
}