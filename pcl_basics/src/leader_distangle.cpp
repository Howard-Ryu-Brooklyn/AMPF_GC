// Copyright (c) 2022 Jonas Mahler

// This file is part of pcl_example.

// pcl_example is free software: you can redistribute it and/or modify it under the terms
// of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.

// pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with Foobar. If not, see <https://www.gnu.org/licenses/>.

#define BOOST_BIND_NO_PLACEHOLDERS

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

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

double getDist(pcl::PointXYZ p_)
{
  return sqrt(pow(p_.x, 2) + pow(p_.y, 2) + pow(p_.z, 2));
}

class LeaderDistAngle : public rclcpp::Node
{
public:
  LeaderDistAngle()
      : Node("leader_distangle")
  {
    declare_parameter("polar_timer_period", 33);       // 30hz
    declare_parameter("point_cloud_timer_period", 10); // 10hz

    get_parameter("polar_timer_period", polar_tp);
    get_parameter("point_cloud_timer_period", point_cloud_tp);

    std::chrono::milliseconds polar_timer_period(polar_tp);
    std::chrono::milliseconds point_cloud_timer_period(point_cloud_tp);

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_pc2", rclcpp::SensorDataQoS());
    polar_pub_ = this->create_publisher<my_interfaces::msg::PolarCoor>("/leader_polar", rclcpp::SensorDataQoS());

    // enable topic statistics
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    options.topic_stats_options.publish_topic = "/pcl_topic_statistics";

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pcl", rclcpp::SensorDataQoS(), std::bind(&LeaderDistAngle::pc_sub_callback, this, _1), options);

    options.topic_stats_options.publish_topic = "/boundingbox_topic_statistics";
    bbox_sub_ = this->create_subscription<my_interfaces::msg::BoundingBox>(
        "/boundingbox", rclcpp::SensorDataQoS(), std::bind(&LeaderDistAngle::bbox_callback, this, _1), options);

    polar_timer_ = this->create_wall_timer(polar_timer_period, std::bind(&LeaderDistAngle::polar_timer_callback, this));
    pc_timer_ = this->create_wall_timer(point_cloud_timer_period, std::bind(&LeaderDistAngle::pointcloud_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "leader_dist angle node has been initialized \n PCL VERSION: %d", PCL_VERSION);
    std::cout << "timer period is shown as below" << std::endl;
    std::cout << "polar timer period: " << polar_timer_period.count() << "[ms]" << std::endl;
    std::cout << "point cloud timer period: " << point_cloud_timer_period.count() << "[ms]" << std::endl;
  }

private:
  void polar_timer_callback()
  {
    polar_msg.header.stamp = this->now();
    polar_pub_->publish(polar_msg);
  }

  void pointcloud_timer_callback()
  {
    cloud_cropped.header.stamp = this->now();
    pc_pub_->publish(cloud_cropped);
  }

  void pc_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // msg information
    // height 512
    // width 896
    // fields x y z rgb -> datatype 7 (=FLOAT32)

    // ROS2PCL
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *cloud);

    auto width = msg->width;

    auto t1 = this->now();
    if (my_box.detected == true)
    {
      pcl::Indices d_area;

      int j = 0;
      auto box_start = my_box.ymin * width + my_box.xmin;
      auto box_end = my_box.ymax * width + my_box.xmax;

      for (unsigned int i = box_start; i < box_end; i++)
      {
        if (i > (my_box.ymin + j) * width + my_box.xmin && i < (my_box.ymin + j) * width + my_box.xmax)
          d_area.push_back(i);

        if (i > (my_box.ymin + j + 1) * width)
          j++;
      }

      // COPY POINTCLOUD OF SPECIFIC INDICES
      pcl::copyPointCloud(*cloud, d_area, *cloud_filtered);

      // point cloud is suitable for using other algorithm in pcl
      pcl::PointCloud<pcl::PointXYZ> cf_pc;
      pcl::fromPCLPointCloud2(*cloud_filtered, cf_pc);
      pcl::Indices index_nan;
      pcl::removeNaNFromPointCloud(cf_pc, cf_pc, index_nan);
      // std::cout << cf_pc << std::endl;

      // get disvector
      std::vector<double> dist_vec;
      for (auto &c : cf_pc)
        dist_vec.push_back(getDist(c));

      // get smallest value and index
      vector<double>::iterator iter_smallest;
      iter_smallest = min_element(dist_vec.begin(), dist_vec.end());
      auto idx_smallest = distance(dist_vec.begin(), iter_smallest);

      // get angle
      auto angle = atan2(cf_pc.at(idx_smallest).y, cf_pc.at(idx_smallest).x);

      polar_msg.x = cf_pc.at(idx_smallest).x;
      polar_msg.y = cf_pc.at(idx_smallest).y;
      polar_msg.r = dist_vec.at(idx_smallest);
      polar_msg.theta = angle;

      // PCL2ROS
      pcl_conversions::fromPCL(*cloud_filtered, cloud_cropped);
      cloud_cropped.header.frame_id = msg->header.frame_id;
    }
    auto t2 = this->now();
    // std::cout << "caluate time: " << (t2.nanoseconds() - t1.nanoseconds()) / 1000000 << "[ms]" << std::endl;
  }

  void bbox_callback(const my_interfaces::msg::BoundingBox::SharedPtr bbmsg)
  {
    my_box.detected = bbmsg->detected;
    my_box.id = bbmsg->id;
    my_box.probability = bbmsg->probability;
    my_box.xmin = bbmsg->xmin;
    my_box.ymin = bbmsg->ymin;
    my_box.xmax = bbmsg->xmax;
    my_box.ymax = bbmsg->ymax;

    // RCLCPP_INFO_STREAM(this->get_logger(), "id: " << bbmsg->id << ", prob: " << bbmsg->probability << ", (xmin, xmax, ymin, ymax): " << bbmsg->xmin << ", " << bbmsg->xmax << ", " << bbmsg->ymin << ", " << bbmsg->ymax);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<my_interfaces::msg::PolarCoor>::SharedPtr polar_pub_;

  rclcpp::TimerBase::SharedPtr polar_timer_;
  rclcpp::TimerBase::SharedPtr pc_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<my_interfaces::msg::BoundingBox>::SharedPtr bbox_sub_;

  sensor_msgs::msg::PointCloud2 cloud_cropped;
  my_interfaces::msg::PolarCoor polar_msg;
  my_interfaces::msg::BoundingBox my_box;

  int polar_tp;
  int point_cloud_tp;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderDistAngle>());
  rclcpp::shutdown();
  return 0;
}