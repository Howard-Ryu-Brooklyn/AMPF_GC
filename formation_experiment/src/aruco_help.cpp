#include "formation_experiment/aruco_help.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

ARUCO_HELP::ARUCO_HELP()
    : Node("aruco_help")
{
    // get parameters position of desired formation
    declare_parameters();

    std::chrono::milliseconds timer_period_ms(timer_period_);

    // declare publisher and timer
    f1_polar_coor_pub_ = this->create_publisher<my_interfaces::msg::PolarCoor>("/f1/polar_coordinate", rclcpp::SensorDataQoS());
    f2_polar_coor_pub_ = this->create_publisher<my_interfaces::msg::PolarCoor>("/f2/polar_coordinate", rclcpp::SensorDataQoS());
    aruco_marker_pub_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("sorted_aruco_markers", rclcpp::SensorDataQoS());
    aruco_marker_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/aruco_markers", rclcpp::SensorDataQoS(), std::bind(&ARUCO_HELP::aruco_marker_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        timer_period_ms, std::bind(&ARUCO_HELP::timer_callback, this));

    geometry_msgs::msg::Pose init_pos;

    // INIT VARIABLES
    for (int i = 0; i < N_AGENT; i++)
    {
        aruco_markers_.marker_ids.push_back(i + 1);
        aruco_markers_.poses.push_back(init_pos);
    }
    // RCLCPP_INFO_STREAM(get_logger(), "posevalue:" << rosidl_generator_traits::to_yaml(init_pos));
    RCLCPP_INFO(this->get_logger(), "ARUCO_HELP node has been initialised");
}

ARUCO_HELP::~ARUCO_HELP()
{
    RCLCPP_INFO(this->get_logger(), "ARUCO_HELP node has been terminated");
}

void ARUCO_HELP::declare_parameters()
{
    // declare and initialize
    this->declare_parameter("timer_period", 100);

    // get param object
    rclcpp::Parameter param_timer_period_ = this->get_parameter("timer_period");

    // get type value
    timer_period_ = this->get_parameter("timer_period").as_int();

    // print parameters
    RCLCPP_INFO(this->get_logger(), "*** DESIRED FORMATION PARAMETERS ARE SHOWN AS BELOW ***");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms]" << std::setw(10) << "*");
}

void ARUCO_HELP::aruco_marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    aruco_markers_.header.stamp = msg->header.stamp;
    // RCLCPP_INFO_STREAM(this->get_logger(), "array_size " << sizeof(msg->marker_ids) / sizeof(long unsigned int));
    for (int i = 0; i < N_AGENT; i++)
    {
        long unsigned int id_idx = msg->marker_ids[i] - 1;
        if (id_idx >= N_AGENT)
            continue;
        aruco_markers_.marker_ids[id_idx] = id_idx;
        aruco_markers_.poses[id_idx].position.x = msg->poses[i].position.x;
        aruco_markers_.poses[id_idx].position.y = msg->poses[i].position.y;
        aruco_markers_.poses[id_idx].position.z = msg->poses[i].position.z;
        aruco_markers_.poses[id_idx].orientation.w = msg->poses[i].orientation.w;
        aruco_markers_.poses[id_idx].orientation.x = msg->poses[i].orientation.x;
        aruco_markers_.poses[id_idx].orientation.y = msg->poses[i].orientation.y;
        aruco_markers_.poses[id_idx].orientation.z = msg->poses[i].orientation.z;
        // RCLCPP_INFO_STREAM(get_logger(), id_idx << ":" << rosidl_generator_traits::to_yaml(aruco_markers_.poses[id_idx]));
    }
    aruco_marker_pub_->publish(aruco_markers_);
}

double ARUCO_HELP::calculate_yaw_from_quat(geometry_msgs::msg::Pose pose)
{
    double w = pose.orientation.w;
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;

    double dcm10 = 2 * (x * y + w * z);
    double dcm00 = w * w + x * x - y * y - z * z;

    double yaw = atan2(dcm10, dcm00);

    return yaw;
}

void ARUCO_HELP::calculate_polar_coor()
{
    // calculate leader polar coordinate with respect to follower 1
    pc_f1_.x = aruco_markers_.poses[0].position.x - aruco_markers_.poses[1].position.x;
    pc_f1_.y = aruco_markers_.poses[0].position.y - aruco_markers_.poses[1].position.y;
    pc_f1_.r = sqrt(pc_f1_.x * pc_f1_.x + pc_f1_.y * pc_f1_.y);
    pc_f1_.los = atan2(pc_f1_.y, pc_f1_.x);

    if (pc_f1_.los < 0)
        pc_f1_.los += 2 * MATH_PI;

    double f1_yaw = calculate_yaw_from_quat(aruco_markers_.poses[1]);

    if (f1_yaw < 0)
        f1_yaw += 2 * MATH_PI;

    pc_f1_.local_angle = f1_yaw;
    pc_f1_.theta = pc_f1_.los - pc_f1_.local_angle;

    // calculate leader polar coordinate with respect to follower 2
    pc_f2_.x = aruco_markers_.poses[0].position.x - aruco_markers_.poses[2].position.x;
    pc_f2_.y = aruco_markers_.poses[0].position.y - aruco_markers_.poses[2].position.y;
    pc_f2_.r = sqrt(pc_f2_.x * pc_f2_.x + pc_f2_.y * pc_f2_.y);
    pc_f2_.los = atan2(pc_f2_.y, pc_f2_.x);

    if (pc_f2_.los < 0)
        pc_f2_.los += 2 * MATH_PI;

    double f2_yaw = calculate_yaw_from_quat(aruco_markers_.poses[2]);

    if (f2_yaw < 0)
        f2_yaw += 2 * MATH_PI;
    pc_f2_.local_angle = f2_yaw;
    pc_f2_.theta = pc_f2_.los - pc_f2_.local_angle;
}

void ARUCO_HELP::timer_callback()
{
    calculate_polar_coor();

    pc_f1_.header.stamp = this->now();
    f1_polar_coor_pub_->publish(pc_f1_);

    pc_f2_.header.stamp = this->now();
    f2_polar_coor_pub_->publish(pc_f2_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARUCO_HELP>());
    rclcpp::shutdown();
    return 0;
}