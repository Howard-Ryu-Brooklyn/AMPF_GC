#include "formation_controller/desired_formation.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

DesiredFormation::DesiredFormation()
    : Node("desired_formation")
{
    // get parameters position of desired formation
    declare_parameters();

    std::chrono::milliseconds timer_period_ms(timer_period_);

    // declare publisher and timer
    publisher_ = this->create_publisher<my_interfaces::msg::DesiredFormation>("/desired_formation", rclcpp::SensorDataQoS());
    timer_ = this->create_wall_timer(
        timer_period_ms, std::bind(&DesiredFormation::timer_callback, this));

    calculate_zs_squared();

    RCLCPP_INFO(this->get_logger(), "DesiredFormation node has been initialised");
}

DesiredFormation::~DesiredFormation()
{
    RCLCPP_INFO(this->get_logger(), "DesiredFormation node has been terminated");
}

void DesiredFormation::declare_parameters()
{
    // declare and initialize
    this->declare_parameter("desired_p1", std::vector<double>{0.2772, 0.2986});
    this->declare_parameter("desired_p2", std::vector<double>{1.3982, 1.7818});
    this->declare_parameter("desired_p3", std::vector<double>{1.9186, 1.0944});
    this->declare_parameter("timer_period", 100);

    // get param object
    rclcpp::Parameter param_desired_p1_ = this->get_parameter("desired_p1");
    rclcpp::Parameter param_desired_p2_ = this->get_parameter("desired_p2");
    rclcpp::Parameter param_desired_p3_ = this->get_parameter("desired_p3");
    rclcpp::Parameter param_timer_period_ = this->get_parameter("timer_period");

    // get type value
    desired_p1_ = this->get_parameter("desired_p1").as_double_array();
    desired_p2_ = this->get_parameter("desired_p2").as_double_array();
    desired_p3_ = this->get_parameter("desired_p3").as_double_array();
    timer_period_ = this->get_parameter("timer_period").as_int();

    // print parameters
    RCLCPP_INFO(this->get_logger(), "*** DESIRED FORMATION PARAMETERS ARE SHOWN AS BELOW ***");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p1: " << desired_p1_[0] << " [m], " << desired_p1_[1] << " [m]" << std::setw(5) << "*");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p2: " << desired_p2_[0] << " [m], " << desired_p2_[1] << " [m]" << std::setw(5) << "*");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "desired_p3: " << desired_p3_[0] << " [m], " << desired_p3_[1] << " [m]" << std::setw(5) << "*");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms]" << std::setw(10) << "*");
}

void DesiredFormation::calculate_zs_squared()
{
    df_.zs12_squared = pow(desired_p1_[0] - desired_p2_[0], 2) + pow(desired_p1_[1] - desired_p2_[1], 2);
    df_.zs13_squared = pow(desired_p1_[0] - desired_p3_[0], 2) + pow(desired_p1_[1] - desired_p3_[1], 2);
    df_.zs23_squared = pow(desired_p2_[0] - desired_p3_[0], 2) + pow(desired_p2_[1] - desired_p3_[1], 2);

    RCLCPP_INFO(this->get_logger(), "*** calcuated zs_squared is shown as below ***");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs12_squared: " << df_.zs12_squared << " [m]" << std::setw(5) << "*");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs13_squared: " << df_.zs13_squared << " [m]" << std::setw(5) << "*");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "zs23_squared: " << df_.zs23_squared << " [m]" << std::setw(5) << "*");
}

void DesiredFormation::timer_callback()
{
    df_.header.stamp = this->now();
    publisher_->publish(df_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DesiredFormation>());
    rclcpp::shutdown();
    return 0;
}