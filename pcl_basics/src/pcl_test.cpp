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
#include "my_yolo_wrapper_interfaces/msg/bounding_box.hpp"
#include "my_yolo_wrapper_interfaces/msg/polar_coor.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

double getDist(pcl::PointXYZ p_)
{
  return sqrt(pow(p_.x, 2) + pow(p_.y, 2) + pow(p_.z, 2));
}

class PCL_node : public rclcpp::Node
{
public:
  PCL_node()
      : Node("pcl_node")
  {
    // r \theta pub, msg 만들기
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_pc2", 10);
    polar_pub_ = this->create_publisher<my_yolo_wrapper_interfaces::msg::PolarCoor>("/leader_polar", 10);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed2i/zed_node/point_cloud/cloud_registered", 10, std::bind(&PCL_node::pc_sub_callback, this, _1));
    bbox_sub_ = this->create_subscription<my_yolo_wrapper_interfaces::msg::BoundingBox>(
        "/my_yolo/BoundingBox", 10, std::bind(&PCL_node::bbox_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "pcl_test node has been initialized \n PCL VERSION: %d", PCL_VERSION);
  }

private:
  void pc_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // msg information
    // height 512
    // width 896
    // fields x y z rgb -> datatype 7 (=FLOAT32)

    auto width = msg->width;

    // ROS2PCL
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *cloud);

    cout << my_box.detected << endl;

    if (my_box.detected == true)
    {
      // processing
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

      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::fromPCLPointCloud2(*cloud_filtered, pc);
      pcl::Indices index_nan;
      pcl::removeNaNFromPointCloud(pc, pc, index_nan);
      std::cout << pc << std::endl;

      // get disvector
      std::vector<double> dist_vec;
      for (auto &c : pc)
        dist_vec.push_back(getDist(c));

      // get smallest value and index
      vector<double>::iterator iter_smallest;

      iter_smallest = min_element(dist_vec.begin(), dist_vec.end());
      auto idx_smallest = distance(dist_vec.begin(), iter_smallest);
      cout << "smallest distance: " << *iter_smallest << endl;
      cout << "index of iterator pointing smallest distance: " << idx_smallest << endl;

      // get angle
      auto angle = atan2(pc.at(idx_smallest).y, pc.at(idx_smallest).x);
      cout << "angle of smallest distance: " << angle * 180 / 3.141592 << endl;

      my_yolo_wrapper_interfaces::msg::PolarCoor polar_msg;
      polar_msg.x = pc.at(idx_smallest).x;
      polar_msg.y = pc.at(idx_smallest).y;
      polar_msg.r = dist_vec.at(idx_smallest);
      polar_msg.theta = angle;

      polar_pub_->publish(polar_msg);

      // PCL2ROS
      sensor_msgs::msg::PointCloud2 cloud_out;
      pcl_conversions::fromPCL(*cloud_filtered, cloud_out);
      cloud_out.header.frame_id = msg->header.frame_id;
      cloud_out.header.stamp = msg->header.stamp;

      pc_pub_->publish(cloud_out);
    }
  }

  void bbox_callback(const my_yolo_wrapper_interfaces::msg::BoundingBox::SharedPtr bbmsg)
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
  rclcpp::Publisher<my_yolo_wrapper_interfaces::msg::PolarCoor>::SharedPtr polar_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<my_yolo_wrapper_interfaces::msg::BoundingBox>::SharedPtr bbox_sub_;

  my_yolo_wrapper_interfaces::msg::BoundingBox my_box;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCL_node>());
  rclcpp::shutdown();
  return 0;
}