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

const double PI = 3.14159265358979;
const double D2R = PI / 180;
const double R2D = 1 / D2R;

enum MODE
{
    STOP,
    FOLLOW_LEADER,
};

class MovingAverageFilter
{
private:
    std::vector<double> buffer;
    size_t windowSize;
    double sum;

public:
    MovingAverageFilter(size_t size) : windowSize(size), sum(0) {}

    double getNextValue(double newValue)
    {
        // Add the new value to the buffer
        buffer.push_back(newValue);
        sum += newValue;

        // If the buffer size exceeds the window size, remove the oldest value
        if (buffer.size() > windowSize)
        {
            sum -= buffer.front();
            buffer.erase(buffer.begin());
        }

        // Calculate and return the current moving average
        return sum / buffer.size();
    }
};

class LowPassFilter
{
public:
    LowPassFilter(float timeConstant, float initialValue = 0.0)
    {
        alpha_ = 1.0 / (1.0 + timeConstant);
        filteredValue_ = initialValue;
    }

    float filter(float inputValue)
    {
        filteredValue_ = alpha_ * inputValue + (1 - alpha_) * filteredValue_;
        return filteredValue_;
    }

    void reset(float initialValue = 0.0)
    {
        filteredValue_ = initialValue;
    }

private:
    float alpha_;
    float filteredValue_;
};

class FOLLOWER1_CONTROLLER : public rclcpp::Node
{
public:
    FOLLOWER1_CONTROLLER();
    ~FOLLOWER1_CONTROLLER();

private:
    // timer, publisher and subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nonfiltered_cmd_vel_pub_;
    rclcpp::Publisher<my_interfaces::msg::Follower1Info>::SharedPtr follower1info_pub_;
    rclcpp::Subscription<my_interfaces::msg::PolarCoor>::SharedPtr leader_position_sub_;
    rclcpp::Subscription<my_interfaces::msg::DesiredFormation>::SharedPtr desired_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr leader_lost_sub_;

    // msgs
    geometry_msgs::msg::Twist cmd_vel_;
    geometry_msgs::msg::Twist nonfiltered_cmd_vel_;
    my_interfaces::msg::Follower1Info f1i_;
    my_interfaces::msg::DesiredFormation df_;
    my_interfaces::msg::PolarCoor pc_;
    std_msgs::msg::Int16 leader_lost_;

    // variable
    int follow_mode_;
    double mag_cmd_;
    bool is_converge_;
    bool is_leader_center_;

    // parameter
    int timer_period_;
    double v_sat_;
    double w_sat_;
    double search_w_;
    double mode_threshold_;
    bool sim_;
    double spf_alpha_;

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