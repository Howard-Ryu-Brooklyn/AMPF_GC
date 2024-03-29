#ifndef FORMATION_CONTROLLER__FOLLOWER2_CONTROLLER_NODE_HPP_
#define FORMATION_CONTROLLER__FOLLOWER2_CONTROLLER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "my_interfaces/msg/polar_coor.hpp"
#include "my_interfaces/msg/desired_formation.hpp"
#include "my_interfaces/msg/uwb.hpp"
#include "my_interfaces/msg/follower2_info.hpp"
#include "my_interfaces/msg/localization.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

const double PI = 3.14159265358979;
const double D2R = PI / 180;
const double R2D = 1 / D2R;

enum MODE
{
    STOP,
    FOLLOW_LEADER,
    FOLLOW_FOLLOWER1
};

using namespace std;

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

struct Localization
{
    double D;
    double zeta1;
    double zeta2[2];
    double zeta1_dot;
    double zeta2_dot[2];
    int alpha;
    int gamma;
    double z_fil;
    double phi_fil[2];
    double z_hat;
    double x_hat[2];
    double x_hat_dot[2];
    double pe_amp;
    double pe_freq;
    int timer_cnt;
    double tol_ts;
    double tol_tf;
    double TOL_TIME;
};

double norm(double vec[2]);
double norm_square(double vec[2]);

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

class FOLLOWER2_CONTROLLER : public rclcpp::Node
{
public:
    FOLLOWER2_CONTROLLER();
    ~FOLLOWER2_CONTROLLER();

private:
    // timer, publisher and subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nonfiltered_cmd_vel_pub_;
    rclcpp::Publisher<my_interfaces::msg::Follower2Info>::SharedPtr follower2info_pub_;
    rclcpp::Publisher<my_interfaces::msg::Localization>::SharedPtr localization_pub_;
    rclcpp::Publisher<my_interfaces::msg::Uwb>::SharedPtr filtered_uwb_pub_;

    rclcpp::Subscription<my_interfaces::msg::PolarCoor>::SharedPtr leader_position_sub_;
    rclcpp::Subscription<my_interfaces::msg::DesiredFormation>::SharedPtr desired_position_sub_;
    rclcpp::Subscription<my_interfaces::msg::Uwb>::SharedPtr uwb_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr leader_lost_sub_;

    // msgs
    geometry_msgs::msg::Twist cmd_vel_;
    geometry_msgs::msg::Twist nonfiltered_cmd_vel_;
    nav_msgs::msg::Odometry odom_;
    my_interfaces::msg::DesiredFormation df_;
    my_interfaces::msg::PolarCoor lpc_;
    my_interfaces::msg::Uwb uwb_;
    my_interfaces::msg::Uwb filtered_uwb_;
    my_interfaces::msg::Follower2Info f2i_;
    my_interfaces::msg::Localization l_;
    std_msgs::msg::Int16 leader_lost_;

    // variable
    unsigned long long int i_;
    int localization_mode_;
    int follow_mode_;
    double tstart_;
    double ts_;
    double d_; //[m]
    double f2_pos_[2];
    double f2_att_;
    Localization L;
    double mode1V_;
    double mode1W_;
    double mode2V_;
    double mode2W_;
    float uwb_pre_;
    double follow_leader_mag_;
    double follow_follower2_mag_;
    bool is_leader_center_;
    bool is_follower1_center_;

    // parameter
    int timer_period_;
    double pe_amp_;
    double pe_freq_;
    double tol_time_;
    double v_sat_;
    double w_sat_;
    double search_w_;
    double mode_threshold_;
    double center_threshold_;
    bool sim_;
    double spf_alpha_;

    // function prototype
    void
    declare_parameters();
    void declare_publisher_and_timer();
    void declare_subscriber();
    void timer_callback();
    void leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg);
    void follower_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg);
    void desired_position_callback(const my_interfaces::msg::DesiredFormation::SharedPtr msg);
    void uwb_callback(const my_interfaces::msg::Uwb::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void leader_lost_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void localize();
    bool localization_stop_condition();
    void localization_pub();
    void gradient_controller();
};

#endif