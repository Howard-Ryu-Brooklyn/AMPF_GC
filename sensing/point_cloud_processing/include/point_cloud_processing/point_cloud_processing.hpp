#ifndef POINT_CLOUD_PROCESSING__POINT_CLOUD_PROCESSING_NODE_HPP_
#define POINT_CLOUD_PROCESSING__POINT_CLOUD_PROCESSING_NODE_HPP_

#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <functional>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "my_interfaces/msg/bounding_box.hpp"
#include "my_interfaces/msg/polar_coor.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

double getDist(pcl::PointXYZ p_);

class PointCloudProcessing : public rclcpp::Node
{
public:
    PointCloudProcessing();
    ~PointCloudProcessing();

private:
    // timers, publishers and subscribers
    rclcpp::TimerBase::SharedPtr polar_coordinate_publishing_timer_;
    rclcpp::TimerBase::SharedPtr point_cloud_publishing_timer_;

    rclcpp::Publisher<my_interfaces::msg::PolarCoor>::SharedPtr polar_coordinate_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<my_interfaces::msg::BoundingBox>::SharedPtr bounding_box_sub_;

    // msgs
    std_msgs::msg::Float32 processing_time_msg_;
    sensor_msgs::msg::PointCloud2 cropped_point_cloud_msg_;
    my_interfaces::msg::PolarCoor polar_coordinate_msg_;
    my_interfaces::msg::BoundingBox bounding_box_msg_;

    // variables

    // parameters
    int polar_coordinate_timer_period_;
    int point_cloud_timer_period_;

    // function prototypes
    void declare_set_parameters();
    void declare_set_publishers_timers_subscribers();
    void polar_coordinate_publishing_timer_callback();
    void point_cloud_publishing_timer_callback();
    void point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void bouding_box_callback(const my_interfaces::msg::BoundingBox::SharedPtr bbmsg);
};

#endif // POINT_CLOUD_PROCESSING__POINT_CLOUD_PROCESSING_NODE_HPP_