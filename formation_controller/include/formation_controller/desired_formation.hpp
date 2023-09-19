#ifndef FORMATION_CONTROLLER__DESIRED_FORMATION_NODE_HPP_
#define FORMATION_CONTROLLER__DESIRED_FORMATION_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/desired_formation.hpp"

using namespace std::chrono_literals;

class DesiredFormation : public rclcpp::Node
{
public:
    DesiredFormation();
    ~DesiredFormation();

private:
    // timer, publisher and subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::DesiredFormation>::SharedPtr publisher_;

    // msgs
    my_interfaces::msg::DesiredFormation df_;

    // variable
    int timer_period_;

    // parameter
    std::vector<double> desired_p1_, desired_p2_, desired_p3_;

    // function prototype
    void declare_parameters();
    void calculate_zs_squared();
    void timer_callback();
};

#endif // FORMATION_CONTROLLER__DESIRED_FORMATION_NODE_HPP_