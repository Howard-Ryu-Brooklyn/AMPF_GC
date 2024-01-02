#include "formation_controller/follower1_controller.hpp"

FOLLOWER1_CONTROLLER::FOLLOWER1_CONTROLLER()
    : Node("follower1_controller")
{
  declare_parameters();
  declare_publisher_and_timer();
  declare_subscriber();

  // initialize variables
  follow_mode_ = FOLLOW_LEADER; // follow leader
  mag_cmd_ = 10;                // arbitarary large valeu for preventing to changing to stop mode at the first time
  is_leader_center_ = false;

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
  this->declare_parameter("v_sat", 0.22);
  this->declare_parameter("w_sat", 2.84);
  this->declare_parameter("search_w", 0.5);
  this->declare_parameter("mode_threshold", 0.1);
  this->declare_parameter("sim", true);
  this->declare_parameter("spf_alpha", 0.1);

  // get param object
  rclcpp::Parameter param_timer_period = this->get_parameter("timer_period");
  rclcpp::Parameter param_v_sat = this->get_parameter("v_sat");
  rclcpp::Parameter param_w_sat = this->get_parameter("w_sat");
  rclcpp::Parameter param_search_w = this->get_parameter("search_w");
  rclcpp::Parameter param_mode_threshold = this->get_parameter("mode_threshold");
  rclcpp::Parameter param_sim = this->get_parameter("sim");
  rclcpp::Parameter param_spf_alpha = this->get_parameter("spf_alpha");

  // get type value
  timer_period_ = param_timer_period.as_int();
  v_sat_ = param_v_sat.as_double();
  w_sat_ = param_w_sat.as_double();
  search_w_ = param_search_w.as_double();
  mode_threshold_ = param_mode_threshold.as_double();
  sim_ = param_sim.as_bool();
  spf_alpha_ = param_spf_alpha.as_double();

  // print parameters
  RCLCPP_INFO(this->get_logger(), "*** FOLLOWER1_CONTROLLER PARAMETERS ARE SHOWN AS BELOW ***");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms]" << std::setw(10) << "*");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "v_sat_: " << v_sat_ << " *");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "w_sat_: " << w_sat_ << " *");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "search_w_: " << search_w_ << " *");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "mode_threshold_: " << mode_threshold_ << " *");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "sim_: " << sim_ << " *");
  RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "spf_alpha_: " << spf_alpha_ << " *");
}

void FOLLOWER1_CONTROLLER::declare_publisher_and_timer()
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  nonfiltered_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/nonfiltered_cmd_vel", 0);
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

  if (follow_mode_ == FOLLOW_LEADER)
  {
    if (leader_lost_.data == 1 || !is_leader_center_)
    {
      // search mode
      cmd_vel_.linear.x = 0;

      if (pc_.theta > 0)
        cmd_vel_.angular.z = search_w_;
      else
        cmd_vel_.angular.z = -search_w_;
    }
    else
    {
      // formation mode
      // calculate gradient based controller input
      f1i_.z12[0] = pc_.r * cos(pc_.theta);
      f1i_.z12[1] = pc_.r * sin(pc_.theta);
      f1i_.e12 = (pow(f1i_.z12[0], 2) + pow(f1i_.z12[1], 2)) - df_.zs12_squared; // scalar error

      // projection rule
      cmd_vel_.linear.x = f1i_.e12 * f1i_.z12[0];
      cmd_vel_.angular.z = f1i_.e12 * f1i_.z12[1];

      nonfiltered_cmd_vel_.linear.x = min(v_sat_, max(-v_sat_, cmd_vel_.linear.x));
      nonfiltered_cmd_vel_.angular.z = min(w_sat_, max(-w_sat_, cmd_vel_.angular.z));

      LowPassFilter spf_v(spf_alpha_, 0.0); // setpoint filter
      LowPassFilter spf_w(spf_alpha_, 0.0); // setpoint filter
      cmd_vel_.linear.x = spf_v.filter(nonfiltered_cmd_vel_.linear.x);
      cmd_vel_.angular.z = spf_w.filter(nonfiltered_cmd_vel_.angular.z);
    }
  }
  else if (follow_mode_ == STOP)
  {
    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 0;
  }

  // revert direction
  if (!sim_)
    cmd_vel_.linear.x = -1 * cmd_vel_.linear.x;

  mag_cmd_ = cmd_vel_.linear.x * cmd_vel_.linear.x + cmd_vel_.angular.z * cmd_vel_.angular.z;
  is_converge_ = mag_cmd_ < mode_threshold_;

  if (is_converge_)
  {
    follow_mode_ = STOP;
    RCLCPP_INFO_ONCE(this->get_logger(), "STOP MODE ACTIVATED");
  }

  f1i_.follow_mode = follow_mode_;
  f1i_.mag_cmd = mag_cmd_;
  f1i_.mode_threshold = mode_threshold_;
  f1i_.is_leader_center = is_leader_center_;

  // pub
  cmd_vel_pub_->publish(cmd_vel_);
  nonfiltered_cmd_vel_pub_->publish(nonfiltered_cmd_vel_);
  f1i_.header.stamp.sec = this->get_clock()->now().seconds();
  f1i_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
  follower1info_pub_->publish(f1i_);
}

void FOLLOWER1_CONTROLLER::leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg)
{
  MovingAverageFilter mv_filter_r(5);
  MovingAverageFilter mv_filter_theta(5);

  pc_.header.stamp = msg->header.stamp;
  pc_.x = msg->x;
  pc_.y = msg->y;

  pc_.r = mv_filter_r.getNextValue(msg->r);
  pc_.theta = mv_filter_theta.getNextValue(msg->theta);

  if (pc_.theta > 0)
  {
    is_leader_center_ = pc_.theta < 5 * D2R;
  }
  else
  {
    is_leader_center_ = pc_.theta > -5 * D2R;
  }
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