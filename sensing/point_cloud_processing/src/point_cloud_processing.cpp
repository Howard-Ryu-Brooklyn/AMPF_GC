#include "point_cloud_processing/point_cloud_processing.hpp"

double getDist(pcl::PointXYZ p_)
{
  return sqrt(pow(p_.x, 2) + pow(p_.y, 2) + pow(p_.z, 2));
}

PointCloudProcessing::PointCloudProcessing()
    : Node("point_cloud_processing")
{
  declare_set_parameters();
  declare_set_publishers_timers_subscribers();

  RCLCPP_INFO(this->get_logger(), "point cloud processing node has been initialised");
}

PointCloudProcessing::~PointCloudProcessing()
{
  RCLCPP_INFO(this->get_logger(), "point cloud processing node has been terminated");
}

void PointCloudProcessing::declare_set_parameters()
{
  this->declare_parameter("polar_timer_period", 33);       // 30hz
  this->declare_parameter("point_cloud_timer_period", 10); // 10hz

  rclcpp::Parameter param_polar_coordinate_timer_period = this->get_parameter("polar_timer_period");
  rclcpp::Parameter param_point_cloud_timer_period = this->get_parameter("point_cloud_timer_period");

  polar_coordinate_timer_period_ = param_polar_coordinate_timer_period.as_int();
  point_cloud_timer_period_ = param_point_cloud_timer_period.as_int();

  RCLCPP_INFO(this->get_logger(), "*** POINT CLOUD PROCESSING HAS BEEN INITIALIZED ***");
  RCLCPP_INFO(this->get_logger(), "PCL VERSION: %d", PCL_VERSION);
  RCLCPP_INFO_STREAM(this->get_logger(), "timer period is shown as below");
  RCLCPP_INFO_STREAM(this->get_logger(), "polar timer period: " << polar_coordinate_timer_period_ << "[ms]");
  RCLCPP_INFO_STREAM(this->get_logger(), "point cloud timer period: " << point_cloud_timer_period_ << "[ms]");
}

void PointCloudProcessing::declare_set_publishers_timers_subscribers()
{

  // PUBLISHER
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropped_point_cloud", rclcpp::SensorDataQoS());
  polar_coordinate_pub_ = this->create_publisher<my_interfaces::msg::PolarCoor>("/poloar_coordinate", rclcpp::SensorDataQoS());
  processing_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/point_cloud_processing_time", rclcpp::SensorDataQoS());

  if (0)
  {
    // enable topic statistics
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    options.topic_stats_options.publish_topic = "/pcl_topic_statistics";
    // SUBSCRIPTION
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pcl", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessing::point_cloud_sub_callback, this, _1), options);

    options.topic_stats_options.publish_topic = "/boundingbox_topic_statistics";
    bounding_box_sub_ = this->create_subscription<my_interfaces::msg::BoundingBox>(
        "/boundingbox", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessing::bouding_box_callback, this, _1), options);

    // TIMER
    polar_coordinate_publishing_timer_ = this->create_wall_timer(std::chrono::milliseconds(polar_coordinate_timer_period_), std::bind(&PointCloudProcessing::polar_coordinate_publishing_timer_callback, this));
    point_cloud_publishing_timer_ = this->create_wall_timer(std::chrono::milliseconds(point_cloud_timer_period_), std::bind(&PointCloudProcessing::point_cloud_publishing_timer_callback, this));
  }

  // SUBSCRIPTION
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pcl", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessing::point_cloud_sub_callback, this, _1));

  bounding_box_sub_ = this->create_subscription<my_interfaces::msg::BoundingBox>(
      "/boundingbox", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessing::bouding_box_callback, this, _1));

  // TIMER
  polar_coordinate_publishing_timer_ = this->create_wall_timer(std::chrono::milliseconds(polar_coordinate_timer_period_), std::bind(&PointCloudProcessing::polar_coordinate_publishing_timer_callback, this));
  point_cloud_publishing_timer_ = this->create_wall_timer(std::chrono::milliseconds(point_cloud_timer_period_), std::bind(&PointCloudProcessing::point_cloud_publishing_timer_callback, this));
}

void PointCloudProcessing::polar_coordinate_publishing_timer_callback()
{
  polar_coordinate_msg_.header.stamp = this->now();
  polar_coordinate_pub_->publish(polar_coordinate_msg_);
}

void PointCloudProcessing::point_cloud_publishing_timer_callback()
{
  cropped_point_cloud_msg_.header.stamp = this->now();
  point_cloud_pub_->publish(cropped_point_cloud_msg_);
}

void PointCloudProcessing::point_cloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
  if (bounding_box_msg_.detected == true)
  {
    pcl::Indices d_area;

    int j = 0;
    auto box_start = bounding_box_msg_.ymin * width + bounding_box_msg_.xmin;
    auto box_end = bounding_box_msg_.ymax * width + bounding_box_msg_.xmax;

    for (unsigned int i = box_start; i < box_end; i++)
    {
      if (i > (bounding_box_msg_.ymin + j) * width + bounding_box_msg_.xmin && i < (bounding_box_msg_.ymin + j) * width + bounding_box_msg_.xmax)
        d_area.push_back(i);

      if (i > (bounding_box_msg_.ymin + j + 1) * width)
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

    polar_coordinate_msg_.x = cf_pc.at(idx_smallest).x;
    polar_coordinate_msg_.y = cf_pc.at(idx_smallest).y;
    polar_coordinate_msg_.r = dist_vec.at(idx_smallest);
    polar_coordinate_msg_.theta = angle;

    // PCL2ROS
    pcl_conversions::fromPCL(*cloud_filtered, cropped_point_cloud_msg_);
    cropped_point_cloud_msg_.header.frame_id = msg->header.frame_id;
  }
  auto t2 = this->now();

  processing_time_msg_.data = (t2.nanoseconds() - t1.nanoseconds()) / 1000000;
  processing_time_pub_->publish(processing_time_msg_);
  // std::cout << "caluate time: " << (t2.nanoseconds() - t1.nanoseconds()) / 1000000 << "[ms]" << std::endl;
}

void PointCloudProcessing::bouding_box_callback(const my_interfaces::msg::BoundingBox::SharedPtr bbmsg)
{
  bounding_box_msg_.detected = bbmsg->detected;
  bounding_box_msg_.id = bbmsg->id;
  bounding_box_msg_.probability = bbmsg->probability;
  bounding_box_msg_.xmin = bbmsg->xmin;
  bounding_box_msg_.ymin = bbmsg->ymin;
  bounding_box_msg_.xmax = bbmsg->xmax;
  bounding_box_msg_.ymax = bbmsg->ymax;

  // RCLCPP_INFO_STREAM(this->get_logger(), "id: " << bbmsg->id << ", prob: " << bbmsg->probability << ", (xmin, xmax, ymin, ymax): " << bbmsg->xmin << ", " << bbmsg->xmax << ", " << bbmsg->ymin << ", " << bbmsg->ymax);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessing>());
  rclcpp::shutdown();
  return 0;
}