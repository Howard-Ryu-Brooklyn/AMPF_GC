#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/desired_formation.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DesiredFormation : public rclcpp::Node
{
public:
    DesiredFormation()
        : Node("desired_formation")
    {
        // get parameters position of desired formation
        handle_parameters();

        std::chrono::milliseconds timer_period_ms(timer_period_);
        // declare publisher and timer
        publisher_ = this->create_publisher<my_interfaces::msg::DesiredFormation>("/desired_formation", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(
            timer_period_ms, std::bind(&DesiredFormation::timer_callback, this));

        // calculate zs12_squared, zs13_squared, zs23_squared
        calculate_zs_squared();
    }

private:
    void handle_parameters()
    {
        this->declare_parameter("desired_p1", std::vector<double>{1.0, 1.0});
        this->declare_parameter("desired_p2", std::vector<double>{2.0, 0.0});
        this->declare_parameter("desired_p3", std::vector<double>{0.0, 3.0});
        this->declare_parameter("timer_period", 100);

        rclcpp::Parameter param_desired_p1_ = this->get_parameter("desired_p1");
        rclcpp::Parameter param_desired_p2_ = this->get_parameter("desired_p2");
        rclcpp::Parameter param_desired_p3_ = this->get_parameter("desired_p3");
        rclcpp::Parameter param_timer_period_ = this->get_parameter("timer_period");

        desired_p1 = this->get_parameter("desired_p1").as_double_array();
        desired_p2 = this->get_parameter("desired_p2").as_double_array();
        desired_p3 = this->get_parameter("desired_p3").as_double_array();
        timer_period_ = this->get_parameter("timer_period").as_int();

        // print parameters via RCLCPP_INFO_STREAM aligned with setw() and setprecision()
        RCLCPP_INFO(this->get_logger(), "*** DESIRED FORMATION PARAMETERS ARE SHOWN AS BELOW ***");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p1: " << desired_p1[0] << " [m], " << desired_p1[1] << " [m] *");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p2: " << desired_p2[0] << " [m], " << desired_p2[1] << " [m] *");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p3: " << desired_p3[0] << " [m], " << desired_p3[1] << " [m] *");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms] *");
    }

    void calculate_zs_squared()
    {
        zs12_squared = pow(desired_p1[0] - desired_p2[0], 2) + pow(desired_p1[1] - desired_p2[1], 2);
        zs13_squared = pow(desired_p1[0] - desired_p3[0], 2) + pow(desired_p1[1] - desired_p3[1], 2);
        zs23_squared = pow(desired_p2[0] - desired_p3[0], 2) + pow(desired_p2[1] - desired_p3[1], 2);

        df.zs12_squared = zs12_squared;
        df.zs13_squared = zs13_squared;
        df.zs23_squared = zs23_squared;

        RCLCPP_INFO(this->get_logger(), "*** calcuated zs**_squared is shown as below ***");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs12_squared: " << zs12_squared << " [m] *");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs13_squared: " << zs13_squared << " [m] *");
        RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs23_squared: " << zs23_squared << " [m] *");
    }

    void timer_callback()
    {
        df.header.stamp = this->now();
        publisher_->publish(df);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::DesiredFormation>::SharedPtr publisher_;

    my_interfaces::msg::DesiredFormation df;

    int timer_period_;
    std::vector<double> desired_p1, desired_p2, desired_p3;
    float zs12_squared, zs13_squared, zs23_squared;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DesiredFormation>());
    rclcpp::shutdown();
    return 0;
}