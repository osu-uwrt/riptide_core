#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <vn/sensors.h>
#include <riptide_msgs2/srv/query_imu_serial.hpp>

using namespace vn::sensors;
using namespace std::placeholders;

class SerialInterface {
    using SerialRequest = riptide_msgs2::srv::QueryImuSerial;

    std::shared_ptr<VnSensor> vs;
    // rclcpp::Node* node;

    rclcpp::Service<SerialRequest>::SharedPtr srv;

public:
    SerialInterface(std::shared_ptr<VnSensor> imu, rclcpp::Node* imuNode) : vs{imu}/*, node{imuNode}*/ {
        // Create services for all serial communication actions
        srv = imuNode->create_service<SerialRequest>("vectornav/config", std::bind(&SerialInterface::gotServiceRequest, this, _1, _2));
    }

private:
    void gotServiceRequest(std::shared_ptr<SerialRequest::Request> request, std::shared_ptr<SerialRequest::Response> response) {
        if(!vs->isConnected())
            response->response = "IMU not connected";

        response->response = vs->transaction(request->request);
    }
};