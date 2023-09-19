#include "formation_controller/follower1_controller.hpp"

FOLLOWER1_CONTROLLER::FOLLOWER1_CONTROLLER()
    : Node("follower1_controller")
{
  declare_parameters();
  declare_publisher_and_timer();
  declare_subscriber();

  RCLCPP_INFO(this->get_logger(), "Follower1 controller node has been initialised");
}

FOLLOWER1_CONTROLLER::~FOLLOWER1_CONTROLLER()
{
  RCLCPP_INFO(this->get_logger(), "Follower1 controller node has been terminated");
}

void FOLLOWER1_CONTROLLER::declare_parameters()
{
  // declare and initialize
  this->declare_parameter("timer_period", 100); // [ms] 10hz

  // get param object
  rclcpp::Parameter param_timer_period = this->get_parameter("timer_period");

  // get type value
  timer_period_ = param_timer_period.as_int();

  // print parameters
  RCLCPP_INFO(this->get_logger(), "*** FOLLOWER1_CONTROLLER PARAMETERS ARE SHOWN AS BELOW ***");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms]" << std::setw(10) << "*");
}

void FOLLOWER1_CONTROLLER::declare_publisher_and_timer()
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  follower1info_pub_ = this->create_publisher<my_interfaces::msg::Follower1Info>("/follower1info", rclcpp::SensorDataQoS());

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_), std::bind(&FOLLOWER1_CONTROLLER::timer_callback, this));
}

void FOLLOWER1_CONTROLLER::declare_subscriber()
{
  // enable topic statistics
  if (0)
  {
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    options.topic_stats_options.publish_topic = "/polar_topic_statistics";

    leader_position_sub_ = this->create_subscription<my_interfaces::msg::PolarCoor>(
        "/leader_polar", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER1_CONTROLLER::leader_position_callback, this, _1), options);

    options.topic_stats_options.publish_topic = "/df_topic_statistics";
    desired_position_sub_ = this->create_subscription<my_interfaces::msg::DesiredFormation>(
        "/desired_formation", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER1_CONTROLLER::desired_position_callback, this, _1), options);
  }

  leader_position_sub_ = this->create_subscription<my_interfaces::msg::PolarCoor>(
      "/polar_coordinate", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER1_CONTROLLER::leader_position_callback, this, _1));

  desired_position_sub_ = this->create_subscription<my_interfaces::msg::DesiredFormation>(
      "/desired_formation", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER1_CONTROLLER::desired_position_callback, this, _1));

  leader_lost_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/lost_leader", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER1_CONTROLLER::leader_lost_callback, this, _1));
}

void FOLLOWER1_CONTROLLER::timer_callback()
{

  if (leader_lost_.data == 1)
  {
    // search mode
    cmd_vel_.linear.x = 0;
    if (pc_.theta > 0)
      cmd_vel_.angular.z = SEARCH_W;
    else
      cmd_vel_.angular.z = -SEARCH_W;
  }
  else
  {
    // formation mode
    // calculate gradient based controller input
    f1i_.z12[0] = pc_.r * cos(pc_.theta);
    f1i_.z12[1] = pc_.r * sin(pc_.theta);
    f1i_.e12 = (pow(f1i_.z12[0], 2) + pow(f1i_.z12[1], 2)) - df_.zs12_squared; // scalar error

    // projection rule
    cmd_vel_.linear.x = f1i_.e12 * pc_.r * cos(pc_.theta);
    cmd_vel_.angular.z = f1i_.e12 * pc_.r * sin(pc_.theta);

    cmd_vel_.linear.x = min(V_SAT, max(-V_SAT, cmd_vel_.linear.x));
    cmd_vel_.angular.z = min(W_SAT, max(-W_SAT, cmd_vel_.angular.z));
  }

  // pub
  cmd_vel_pub_->publish(cmd_vel_);
  f1i_.header.stamp.sec = this->get_clock()->now().seconds();
  f1i_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
  follower1info_pub_->publish(f1i_);
}

void FOLLOWER1_CONTROLLER::leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg)
{
  pc_.header.stamp = msg->header.stamp;
  pc_.x = msg->x;
  pc_.y = msg->y;
  pc_.r = msg->r;
  pc_.theta = msg->theta;
}

void FOLLOWER1_CONTROLLER::desired_position_callback(const my_interfaces::msg::DesiredFormation::SharedPtr msg)
{
  df_.header.stamp = msg->header.stamp;
  df_.zs12_squared = msg->zs12_squared;
}

void FOLLOWER1_CONTROLLER::leader_lost_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
  leader_lost_.data = msg->data;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FOLLOWER1_CONTROLLER>());
  rclcpp::shutdown();
  return 0;
}