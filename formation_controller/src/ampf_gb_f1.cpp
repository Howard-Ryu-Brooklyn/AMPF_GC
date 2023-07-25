#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_interfaces/msg/polar_coor.hpp"
#include "my_interfaces/msg/desired_formation.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AMPF_GB : public rclcpp::Node
{
public:
  AMPF_GB()
      : Node("AMPF_GB")
  {
    declare_parameter("timer_period", 100); // [ms] 10hz
    get_parameter("timer_period", timer_period_);

    std::chrono::milliseconds timer_period_ms(timer_period_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    options.topic_stats_options.publish_topic = "/polar_topic_statistics";

    leader_position_sub_ = this->create_subscription<my_interfaces::msg::PolarCoor>(
        "/leader_polar", rclcpp::SensorDataQoS(), std::bind(&AMPF_GB::leader_position_callback, this, _1), options);

    options.topic_stats_options.publish_topic = "/df_topic_statistics";
    desired_position_sub_ = this->create_subscription<my_interfaces::msg::DesiredFormation>(
        "/desired_formation", rclcpp::SensorDataQoS(), std::bind(&AMPF_GB::desired_position_callback, this, _1), options);

    timer_ = this->create_wall_timer(
        timer_period_ms, std::bind(&AMPF_GB::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double e12, z12[2];
    z12[0] = pc.x;
    z12[1] = pc.y;
    e12 = (pow(z12[0], 2) + pow(z12[1], 2)) - df.zs12_squared;

    double grad_input[2];
    grad_input[0] = e12 * z12[0];
    grad_input[1] = e12 * z12[1];

    geometry_msgs::msg::Twist cmd_vel;
    // projection rule
    cmd_vel.linear.x = grad_input[0];
    cmd_vel.angular.z = grad_input[1];
    publisher_->publish(cmd_vel);
  }

  void leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg)
  {
    pc.x = msg->x;
    pc.y = msg->y;
    pc.r = msg->r;
    pc.theta = msg->theta;
  }

  void desired_position_callback(const my_interfaces::msg::DesiredFormation::SharedPtr msg)
  {
    df.zs12_squared = msg->zs12_squared;
    // df.zs13_squared = msg->zs13_squared;
    // df.zs23_squared = msg->zs23_squared;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<my_interfaces::msg::PolarCoor>::SharedPtr leader_position_sub_;
  rclcpp::Subscription<my_interfaces::msg::DesiredFormation>::SharedPtr desired_position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  my_interfaces::msg::DesiredFormation df;
  my_interfaces::msg::PolarCoor pc;

  int timer_period_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AMPF_GB>());
  rclcpp::shutdown();
  return 0;
}