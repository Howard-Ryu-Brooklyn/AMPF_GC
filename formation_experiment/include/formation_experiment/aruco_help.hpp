#ifndef FORMATION_EXPERIMENT__ARUCO_HELP_NODE_HPP_
#define FORMATION_EXPERIMENT__ARUCO_HELP_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/polar_coor.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

#define MATH_PI (double)(3.14159265358979)
#define N_AGENT (int)(3)

class ARUCO_HELP : public rclcpp::Node
{
public:
    ARUCO_HELP();
    ~ARUCO_HELP();

private:
    // timer, publisher and subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::PolarCoor>::SharedPtr f1_polar_coor_pub_;
    rclcpp::Publisher<my_interfaces::msg::PolarCoor>::SharedPtr f2_polar_coor_pub_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_pub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;

    // msgs
    my_interfaces::msg::PolarCoor pc_f1_;
    my_interfaces::msg::PolarCoor pc_f2_;
    ros2_aruco_interfaces::msg::ArucoMarkers aruco_markers_;

    // variable
    int timer_period_;

    // parameter

    // function prototype
    void declare_parameters();
    void calculate_polar_coor();
    double calculate_yaw_from_quat(geometry_msgs::msg::Pose pose);
    void timer_callback();
    void aruco_marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
};

#endif // FORMATION_EXPERIMENT__ARUCO_HELP_NODE_HPP_