//ekf that allows for control inputs and damping models

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "riptide_msgs2/msg/depth.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nortek_dvl_msgs/msg/dvl.hpp"
#include "Eigen/Dense"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <vector>
#include <cstring>

#define SIMULATION_CONFIGURATION_SUBPATH "/config/talos.yaml"
#define EKF_CONFIGURATION_SUBPATH "/cfc/talos_ekf.yaml"

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef Eigen::Matrix<double, 12, 1> Vector12;
typedef Eigen::Matrix<double, 12, 12> Matrix12;
typedef Eigen::Matrix<double, 12, 6> Matrix12_6;
typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 1> Vector3;

struct RobotProperties{

    //native parameters
    double mass; // mass
    std::vector<double> inertia; //intertia
    std::vector<double> drag_cofficients; // drag coefficents
    double net_bouyancy; //how floaty or sinky
    std::vector<double> com; // the center of mass of the AUV
    std::vector<double> cob; // the center of bouyancy of the AUV


};

struct SensorConfiguration{
    //NOTE: All sensor data is expected in body frame

    //which dof to apply the sensor data to
    std::vector<bool> apply_mat;

    //how many measurements to average in between application cycles
    double queue = 0;

    //if the linear accelleration includes a gravity component that must be removed
    bool includes_gravitational_accel = false;

    //topic the sensor publishes on
    std::string topic;

    //the index which this config lives at
    int index;
    
};

class EKF : public rclcpp::Node {
public:
    EKF() : Node("EKF") {
        load_parameters();

        auto qos = rclcpp::SensorDataQoS();

        //init subs
        sub_control_wrench = this->create_subscription<geometry_msgs::msg::Twist>("controller/requested_force", qos, std::bind(&EKF::control_force_cb, this, _1));

        // init timers
        this->param_refresh_timer = this->create_wall_timer(1000ms, std::bind(&EKF::refresh_parameters, this));

        state = Vector12::Zero();
        last_state_update = get_ros2_time();

        control_forces_b = Vector3::Zero();
        incoming_control_forces_b = Vector3::Zero();
        control_torques_b = Vector3::Zero();
        incoming_control_torques_b = Vector3::Zero();
        last_control_update = get_ros2_time();
    }

private:

    //ekf parameters
    RobotProperties params;
    std::vector<SensorConfiguration> sensor_configs;

    //sensor callbacks

    //physical properties
    double gravitational_constant = 9.81;

    //state
    Vector12 state;
    double last_state_update;

    //control input
    Vector3 control_forces_b;
    Vector3 incoming_control_forces_b;
    Vector3 control_torques_b;
    Vector3 incoming_control_torques_b;
    double last_control_update;

    rclcpp::TimerBase::SharedPtr param_refresh_timer;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_control_wrench;
    
    void linearize_model(){

        //State Mat
        //x pos
        //x vel
        //y pos
        //y vel
        //z pos
        //z vel
        //roll pos
        //roll vel
        //pitch pos
        //pitch vel
        //yaw pos
        //yaw vel
        
        double current_time = get_ros2_time();
        double dt = current_time - last_state_update;

        //do some runge kutta-ing
        //calculate this linearization's world to body frame rotation
        Matrix3 world_to_body_1 = rotFromEuler(state[6], state[8], state[10]);
        //calculate the state derivative at the begining
        Vector12 d_state1 = formulate_a_matrix() * state + formulate_b_matrix() * get_averaged_control_wrench(current_time, world_to_body_1) + calculate_d_bouyancy(world_to_body_1);

        //calculate the state derivative at the middle based on the beginning
        Vector12 state2 = state + d_state1 * dt / 2;
        //calculate this linearization's world to body frame rotation
        Matrix3 world_to_body_2 = rotFromEuler(state2[6], state2[8], state2[10]);
        Vector12 d_state2 = formulate_a_matrix() * state2 + formulate_b_matrix() * get_averaged_control_wrench(current_time, world_to_body_2) + calculate_d_bouyancy(world_to_body_2);

        //calculate the state derivative at the middle based on the middle
        Vector12 state3 = state + d_state2 * dt / 2;
        //calculate this linearization's world to body frame rotation
        Matrix3 world_to_body_3 = rotFromEuler(state3[6], state3[8], state3[10]);
        Vector12 d_state3 = formulate_a_matrix() * state3 + formulate_b_matrix() * get_averaged_control_wrench(current_time, world_to_body_3) + calculate_d_bouyancy(world_to_body_3);

        //calculate the state derivative at the end based on the middle
        Vector12 state4 = state + d_state3 * dt;
        //calculate this linearization's world to body frame rotation
        Matrix3 world_to_body_4 = rotFromEuler(state3[6], state3[8], state3[10]);
        Vector12 d_state4 = formulate_a_matrix() * state4 + formulate_b_matrix() * get_averaged_control_wrench(current_time, world_to_body_4) + calculate_d_bouyancy(world_to_body_4);

        //update the system state
        this->state = this->state + (dt / 6.0) * (d_state1 + d_state2 * 2 + d_state3 * 2 + d_state4);

        //update last times
        last_state_update = current_time;
        reset_control_averaging(current_time);
    }

    Matrix12 formulate_a_matrix(){
        Matrix12 A = Matrix12::Zero();

        //set all of the position integrators
        A(0, 1) = 1;
        A(2, 3) = 1;
        A(4, 5) = 1;
        A(6, 7) = 1;
        A(8, 9) = 1;
        A(10, 11) = 1;

        //set all the drag terms here once real drag is loaded

        return A;
    }

    Matrix12_6 formulate_b_matrix(){
        //since the control inputs is a wrench, this is trivial
        Matrix12_6 B;


        //NOTE: Factoring the interia of the system in the control wrench update cb as the inertias are in body frame
        B(1, 0) = 1.0;
        B(3, 1) = 1.0;
        B(5, 2) = 1.0;
        B(7, 3) = 1.0;
        B(9, 4) = 1.0;
        B(11, 5) = 1.0;

        return B;
    }

    Vector12 calculate_d_bouyancy(Matrix3 world_to_body){
        //calulate the expected state change due to change in bouyancy

        //calculate the vector between com and cob
        Vector3 com;
        com << params.com[0], params.com[1], params.com[2];
        Vector3 cob;
        cob << params.cob[0], params.cob[1], params.cob[2];
        Vector3 com_cob_vector = world_to_body.transpose() * (cob - com);

        //calculate the world frame bouyant torque
        Vector3 bouyancy;
        bouyancy << 0, 0, params.net_bouyancy;
        Vector3 bouyancy_torque_w = com_cob_vector.cross(bouyancy);

        //calculate the body frame bouyant torque
        Vector3 bouyancy_torque_b = world_to_body * bouyancy_torque_w;
        Vector3 inertia_vector_b;
        inertia_vector_b << params.inertia[0], params.inertia[1], params.inertia[2];
        Vector3 bouyancy_rot_accel_w = world_to_body.transpose * (bouyancy_torque_b / inertia_vector_b);

        Vector12 bouyancy_state_change;
        bouyancy_state_change << 0, 0, 0, 0, 0, params.net_bouyancy / params.mass,
                                0, bouyancy_rot_accel_w[0], 0, bouyancy_rot_accel_w[1], 0, 0;

        return bouyancy_state_change;
    }

    Matrix3 rotFromEuler(double roll_angle, double pitch_angle, double yaw_angle){
        //whos a good LLM

        // Create AngleAxis objects for each Euler angle
        Eigen::AngleAxisd roll_angle_axis(roll_angle, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle_axis(pitch_angle, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle_axis(yaw_angle, Eigen::Vector3d::UnitZ());

        // Convert AngleAxis objects to rotation matrices
        Eigen::Matrix3d rotation_matrix_roll = roll_angle_axis.toRotationMatrix();
        Eigen::Matrix3d rotation_matrix_pitch = pitch_angle_axis.toRotationMatrix();
        Eigen::Matrix3d rotation_matrix_yaw = yaw_angle_axis.toRotationMatrix();

        // Combine rotation matrices using matrix multiplication (in the correct order)
        Eigen::Matrix3d rotation_matrix = rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll;

        return rotation_matrix;

    }

    void load_parameters(){
        //get the configuration directory
        std::string share_dir_path = ament_index_cpp::get_package_share_directory("riptide_descriptions2");
        
        //load in simulation yaml
        YAML::Node config = YAML::LoadFile(share_dir_path + SIMULATION_CONFIGURATION_SUBPATH);

        //load physical params into params object
        params.mass = config["mass"].as<double>();
        params.inertia = config["inertia"].as<std::vector<double>>();
        params.com = config["com"].as<std::vector<double>>();
        
        //calculate the cob
        std::vector<double> ff_wrench = config["controller"]["feed_forward"]["base_wrench"].as<std::vector<double>>(); //factor in autoff here
        params.net_bouyancy = -ff_wrench[2];
        params.cob.resize(3, 0);
        params.cob[2] = params.com[2]; // assume com z to be the same as the com 
        params.cob[0] = -ff_wrench[3] / params.net_bouyancy + params.com[0];
        params.cob[1] = -ff_wrench[4] / params.net_bouyancy + params.com[1];

        //replace with actual drag
        params.drag_cofficients.resize(36, 0);


        RCLCPP_INFO(this->get_logger(), "Loaded interia %f", params.inertia[0]);
    }

    void load_sensor_config(){
        //load in the sensor configuration from the config file
        
        //get the configuration directory
        std::string share_dir_path = ament_index_cpp::get_package_share_directory("riptide_hardware2");

        //load in simulation yaml
        YAML::Node config = YAML::LoadFile(share_dir_path + EKF_CONFIGURATION_SUBPATH);

        YAML::Node sensor_config = config["/**/ekf_localization_node"];

        //read in the gravitational constat
        try{
            gravitational_constant = config["gravitational_acceleration"];
        }catch{
            RCLCPP_INFO(this->get_logger(), "Could not load in a gravitation constant from the configuration file! Defaulting to %f", gravitational_constant);
        }

        //look for imus






        
    }

    //callback to refresh control inputs
    void control_force_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
        //update the control wrench
        
        double current_time = get_ros2_time();

        //perform averaging
        double time_ratio = (current_time - last_control_update) / (current_time - last_state_update);
        control_forces_b = time_ratio * incoming_control_forces_b + control_forces_b * (1 - time_ratio);
        control_torques_b = time_ratio * incoming_control_torques_b + control_torques_b * (1 - time_ratio);
        
        //set the incoming data
        incoming_control_forces_b << msg->linear.x / params.mass, msg->linear.y / params.mass, msg->linear.z / params.mass;
        incoming_control_torques_b << msg->angular.x / params.inertia[0], msg->angular.y / params.inertia[1], msg->angular.z / params.inertia[2];

        last_control_update = current_time;
    }

    Vector12 get_averaged_control_wrench(double current_time, Matrix3 world_to_body){
        
        //perform averaging
        double time_ratio = (current_time - last_control_update) / (current_time - last_state_update);
        Vector3 forces_b = time_ratio * incoming_control_forces_b + control_forces_b * (1 - time_ratio);
        Vector3 torques_b = time_ratio * incoming_control_torques_b + control_torques_b * (1 - time_ratio);

        //change to world frame
        Vector3 forces_w = world_to_body.transpose() * forces_b;
        Vector3 torques_w = world_to_body.transpose() * torques_b;

        //update control wrench vector
        Vector12 cw;
        cw << 0, forces_w[0],
                        0, forces_w[1],
                        0, forces_w[2],
                        0, torques_w[0],
                        0, torques_w[1],
                        0, torques_w[2];

        return cw;
    }

    void reset_control_averaging(double current_time){
        //reset the control averaging for the next timestep
        control_forces_b = Vector3::Zero();
        control_torques_b = Vector3::Zero();
        last_control_update = current_time;
    }

    void refresh_parameters(){
       
    }

    double get_ros2_time()
    {
        rclcpp::Time now = this->get_clock()->now();

        // Combine seconds and nanoseconds into a single float (fractional seconds)
        double ns = now.nanoseconds();

        return ns / 1000000000.0;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}