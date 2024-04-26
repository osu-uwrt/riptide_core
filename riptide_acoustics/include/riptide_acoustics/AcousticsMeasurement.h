#ifndef ACOUSTICS_MEASUREMENT
#define ACOUSTICS_MEASUREMENT

#include <vector>

#include "geometry_msgs/msg/transform.hpp"


class AcousticsMeasurement{
    public:
        AcousticsMeasurement(geometry_msgs::msg::Transform port, geometry_msgs::msg::Transform starboard, double Delta_T, double Timestamp);

    private:
        //position and orientation - quaternion x,y,z,w - of the pods when a measurement happens
        std::vector<float> port_position;
        std::vector<float> port_orientation;
        std::vector<float> starboard_position;
        std::vector<float> starboard_orientation;

        //the time between the pods recieving the acoustic pulse
        //negative for port first, positive for starboard first
        double delta_t;

        //timestamp for when measurement happened
        double timestamp;

        
};

#endif