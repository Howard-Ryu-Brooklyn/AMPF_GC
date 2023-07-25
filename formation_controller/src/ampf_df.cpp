#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
        declare_parameter("zs12", 1.0);
        declare_parameter("zs13", 1.0);
        declare_parameter("zs23", 1.0);
        declare_parameter("timer_period", 100);

        get_parameter("zs12", zs12);
        get_parameter("zs13", zs13);
        get_parameter("zs23", zs23);
        get_parameter("timer_period", timer_period_);

        std::chrono::milliseconds timer_period_ms(timer_period_);

        publisher_ = this->create_publisher<my_interfaces::msg::DesiredFormation>("/desired_formation", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(
            timer_period_ms, std::bind(&DesiredFormation::timer_callback, this));
    }

private:
    void timer_callback()
    {
        df.zs12_squared = zs12;
        df.zs13_squared = zs13;
        df.zs23_squared = zs23;
        df.header.stamp = this->now();
        publisher_->publish(df);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_interfaces::msg::DesiredFormation>::SharedPtr publisher_;

    my_interfaces::msg::DesiredFormation df;
    float zs12, zs13, zs23;
    int timer_period_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DesiredFormation>());
    rclcpp::shutdown();
    return 0;
}