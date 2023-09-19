#ifndef FORMATION_CONTROLLER__FOLLOWER1_CONTROLLER_NODE_HPP_
#define FORMATION_CONTROLLER__FOLLOWER1_CONTROLLER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_interfaces/msg/polar_coor.hpp"
#include "my_interfaces/msg/desired_formation.hpp"
#include "my_interfaces/msg/follower1_info.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

#define V_SAT (double)(0.22)
#define W_SAT (double)(2.84)
#define SEARCH_W (double)(1.0)

class FOLLOWER1_CONTROLLER : public rclcpp::Node
{
public:
    FOLLOWER1_CONTROLLER();
    ~FOLLOWER1_CONTROLLER();

private:
    // timer, publisher and subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<my_interfaces::msg::Follower1Info>::SharedPtr follower1info_pub_;
    rclcpp::Subscription<my_interfaces::msg::PolarCoor>::SharedPtr leader_position_sub_;
    rclcpp::Subscription<my_interfaces::msg::DesiredFormation>::SharedPtr desired_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr leader_lost_sub_;

    // msgs
    geometry_msgs::msg::Twist cmd_vel_;
    my_interfaces::msg::Follower1Info f1i_;
    my_interfaces::msg::DesiredFormation df_;
    my_interfaces::msg::PolarCoor pc_;
    std_msgs::msg::Int16 leader_lost_;

    // variable

    // parameter
    int timer_period_;

    // function prototype
    void declare_parameters();
    void declare_publisher_and_timer();
    void declare_subscriber();
    void timer_callback();
    void leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg);
    void desired_position_callback(const my_interfaces::msg::DesiredFormation::SharedPtr msg);
    void leader_lost_callback(const std_msgs::msg::Int16::SharedPtr msg);
};

#endif