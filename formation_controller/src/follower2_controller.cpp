#include "formation_controller/follower2_controller.hpp"

double norm(double vec[2])
{
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}

double norm_square(double vec[2])
{
    return vec[0] * vec[0] + vec[1] * vec[1];
}

FOLLOWER2_CONTROLLER::FOLLOWER2_CONTROLLER()
    : Node("follower2_controller")
{

    declare_parameters();
    declare_publisher_and_timer();
    declare_subscriber();

    tstart_ = this->get_clock()->now().seconds();
    localization_mode_ = 1;
    follow_mode_ = 2;
    i_ = 0;
    d_ = 0.15; //[m]
    mode1V_ = 0;
    mode1W_ = 0;
    mode2V_ = 0;
    mode2W_ = 0;

    L = {
        0.0,
        0.0,
        {0.0, 0.0},
        0.0,
        {0.0, 0.0},
        1, // alpha
        1, // gamma
        0.0,
        {0.0, 0.0},
        0.0,

        {0.0, 0.0},
        {0.0, 0.0},
        pe_amp_,  // pe_amp
        pe_freq_, // pe_freq
        0,
        0,
        0.0,
        0.0,
        tol_time_,              // TOL_TIME
        localization_tolerance_ // LOCALIZATION_TOLERANCE
    };

    RCLCPP_INFO(this->get_logger(), "Follower2 controller node has been initialised");
}

FOLLOWER2_CONTROLLER::~FOLLOWER2_CONTROLLER()
{
    RCLCPP_INFO(this->get_logger(), "Follower2 controller node has been terminated");
}

void FOLLOWER2_CONTROLLER::declare_parameters()
{
    // declare and initialize
    this->declare_parameter("timer_period", 10); // [ms] 10hz
    this->declare_parameter("pe_amp", 0.0);
    this->declare_parameter("pe_freq", 0.0);
    this->declare_parameter("tol_time", 0.0);
    this->declare_parameter("localization_tolerance", 0.0);

    // get param object
    rclcpp::Parameter param_timer_period = this->get_parameter("timer_period");
    rclcpp::Parameter param_pe_amp = this->get_parameter("pe_amp");
    rclcpp::Parameter param_pe_freq = this->get_parameter("pe_freq");
    rclcpp::Parameter param_tol_time = this->get_parameter("tol_time");
    rclcpp::Parameter param_localization_tolerance = this->get_parameter("localization_tolerance");

    // get type value
    timer_period_ = param_timer_period.as_int();
    ts_ = (double)timer_period_ / 1000;
    pe_amp_ = param_pe_amp.as_double();
    pe_freq_ = param_pe_freq.as_double();
    tol_time_ = param_tol_time.as_double();
    localization_tolerance_ = param_localization_tolerance.as_double();

    // print parameters
    RCLCPP_INFO(this->get_logger(), "*** FOLLOWER2_CONTROLLER PARAMETERS ARE SHOWN AS BELOW ***");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "timer_period: " << timer_period_ << "[ms] *");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "pe_amp_: " << pe_amp_ << " *");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "pe_freq_: " << pe_freq_ << "[hz] *");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "tol_time_: " << tol_time_ << "[sec] *");
    RCLCPP_INFO_STREAM(this->get_logger(), "*" << std::setw(20) << "localization_tolerance_: " << localization_tolerance_ << " *");
}

void FOLLOWER2_CONTROLLER::declare_publisher_and_timer()
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 0);
    follower2info_pub_ = this->create_publisher<my_interfaces::msg::Follower2Info>("/follower2info", rclcpp::SensorDataQoS());
    localization_pub_ = this->create_publisher<my_interfaces::msg::Localization>("/localization", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_period_), std::bind(&FOLLOWER2_CONTROLLER::timer_callback, this));
}

void FOLLOWER2_CONTROLLER::declare_subscriber()
{
    // enable topic statistics
    if (0)
    {
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
        options.topic_stats_options.publish_period = std::chrono::seconds(10);
        options.topic_stats_options.publish_topic = "/polar_topic_statistics";

        leader_position_sub_ = this->create_subscription<my_interfaces::msg::PolarCoor>(
            "/leader_polar", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::leader_position_callback, this, _1), options);

        options.topic_stats_options.publish_topic = "/df_topic_statistics";
        desired_position_sub_ = this->create_subscription<my_interfaces::msg::DesiredFormation>(
            "/desired_formation", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::desired_position_callback, this, _1), options);

        uwb_sub_ = this->create_subscription<my_interfaces::msg::Uwb>(
            "/uwb", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::uwb_callback, this, _1), options);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::odom_callback, this, _1));
    }

    leader_position_sub_ = this->create_subscription<my_interfaces::msg::PolarCoor>(
        "/polar_coordinate", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::leader_position_callback, this, _1));

    desired_position_sub_ = this->create_subscription<my_interfaces::msg::DesiredFormation>(
        "/desired_formation", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::desired_position_callback, this, _1));

    uwb_sub_ = this->create_subscription<my_interfaces::msg::Uwb>(
        "/uwb", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::uwb_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::odom_callback, this, _1));

    leader_lost_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/lost_leader", rclcpp::SensorDataQoS(), std::bind(&FOLLOWER2_CONTROLLER::leader_lost_callback, this, _1));
}

void FOLLOWER2_CONTROLLER::timer_callback()
{
    i_++;

    if (localization_mode_ == 1) // localization
    {
        localize();

        if (localization_stop_condition())
            localization_mode_ = 0;

        localization_pub();

        // feedback linearziation input
        cmd_vel_.linear.x = cos(f2_att_) * L.pe_amp * sin(L.pe_freq * i_ * ts_) + sin(f2_att_) * L.pe_amp * cos(2 * L.pe_freq * i_ * ts_);
        cmd_vel_.angular.z = -1 / d_ * sin(f2_att_) * L.pe_amp * sin(L.pe_freq * i_ * ts_) + 1 / d_ * cos(f2_att_) * L.pe_amp * cos(2 * L.pe_freq * i_ * ts_);
    }
    else
    {
        gradient_controller();
    }

    l_.header.stamp = this->now();
    localization_pub_->publish(l_);

    cmd_vel_.linear.x = min(V_SAT, max(-V_SAT, cmd_vel_.linear.x));
    cmd_vel_.angular.z = min(W_SAT, max(-W_SAT, cmd_vel_.angular.z));
    cmd_vel_pub_->publish(cmd_vel_);

    f2i_.header.stamp = this->now();
    f2i_.localization_mode = localization_mode_;
    f2i_.follow_mode = follow_mode_;
    follower2info_pub_->publish(f2i_);
}

void FOLLOWER2_CONTROLLER::gradient_controller()
{
    if (leader_lost_.data == 1 && follow_mode_ == FOLLOW_LEADER)
    {
        // search mode
        cmd_vel_.linear.x = 0;
        if (lpc_.theta > 0)
            cmd_vel_.angular.z = SEARCH_W;
        else
            cmd_vel_.angular.z = -SEARCH_W;
    }
    else
    {
        // formation controller
        f2i_.z13[0] = lpc_.r * cos(lpc_.theta);
        f2i_.z13[1] = lpc_.r * sin(lpc_.theta);
        f2i_.e13 = (pow(f2i_.z13[0], 2) + pow(f2i_.z13[1], 2)) - df_.zs13_squared; // scalar error

        // update with estimated value
        f2i_.z23[0] = L.x_hat[0] - f2_pos_[0];
        f2i_.z23[1] = L.x_hat[1] - f2_pos_[1];
        f2i_.e23 = (pow(f2i_.z23[0], 2) + pow(f2i_.z23[1], 2)) - df_.zs23_squared; // scalar error

        double r = sqrt(f2i_.z23[0] * f2i_.z23[0] + f2i_.z23[1] * f2i_.z23[1]);
        double theta = atan2(f2i_.z23[1], f2i_.z23[0]);

        mode1V_ = f2i_.e13 * lpc_.r * cos(lpc_.theta);
        mode1W_ = f2i_.e13 * lpc_.r * sin(lpc_.theta);

        mode2V_ = f2i_.e23 * r * cos(theta);
        mode2W_ = f2i_.e23 * r * sin(theta);

        if (follow_mode_ == FOLLOW_LEADER)
        {
            // follow leader
            cmd_vel_.linear.x = mode1V_;
            cmd_vel_.angular.z = mode1W_;
        }
        else if (follow_mode_ == FOLLOW_FOLLOWER1)
        {
            // follow follower2
            cmd_vel_.linear.x = mode2V_;
            cmd_vel_.angular.z = mode2W_;
        }
        else if (follow_mode_ == STOP)
        {
            //  Stop
            cmd_vel_.linear.x = 0;
            cmd_vel_.angular.z = 0;
        }
    }

    int is_Total_Converged = sqrt(mode1V_ * mode1V_ + mode1W_ * mode1W_) < SUBMODE_TRESHOLD && sqrt(mode2V_ * mode2V_ + mode2W_ * mode2W_) < SUBMODE_TRESHOLD;
    int is_Mode_Converged = sqrt(cmd_vel_.linear.x * cmd_vel_.linear.x + cmd_vel_.angular.z * cmd_vel_.angular.z) < SUBMODE_TRESHOLD && follow_mode_ != STOP;

    // Submode stop condition
    if (is_Total_Converged)
    {
        follow_mode_ = STOP;
    }
    // Change submode
    else if (is_Mode_Converged)
    {
        if (follow_mode_ == FOLLOW_LEADER)
            follow_mode_ = FOLLOW_FOLLOWER1;
        else if (follow_mode_ == FOLLOW_FOLLOWER1)
            follow_mode_ = FOLLOW_LEADER;
    }
}

void FOLLOWER2_CONTROLLER::localize()
{

    L.zeta1_dot = -L.alpha * L.zeta1 + 0.5 * (norm_square(f2_pos_) - L.D * L.D);
    L.zeta2_dot[0] = -L.alpha * L.zeta2[0] + f2_pos_[0];
    L.zeta2_dot[1] = -L.alpha * L.zeta2[1] + f2_pos_[1];

    L.zeta1 = L.zeta1 + L.zeta1_dot * ts_;
    L.zeta2[0] = L.zeta2[0] + L.zeta2_dot[0] * ts_;
    L.zeta2[1] = L.zeta2[1] + L.zeta2_dot[1] * ts_;

    L.z_fil = L.zeta1_dot;
    L.phi_fil[0] = L.zeta2_dot[0];
    L.phi_fil[1] = L.zeta2_dot[1];
    L.z_hat = L.x_hat[0] * L.phi_fil[0] + L.x_hat[1] * L.phi_fil[1];

    L.x_hat_dot[0] = L.gamma * (L.z_fil - L.z_hat) * L.phi_fil[0];
    L.x_hat_dot[1] = L.gamma * (L.z_fil - L.z_hat) * L.phi_fil[1];
    L.x_hat[0] = L.x_hat[0] + L.x_hat_dot[0] * ts_;
    L.x_hat[1] = L.x_hat[1] + L.x_hat_dot[1] * ts_;

    L.timer_cnt++;
}

bool FOLLOWER2_CONTROLLER::localization_stop_condition()
{
    if (norm(L.x_hat_dot) < L.LOCALIZATION_TOLERLANCE && norm(L.x_hat_dot) != 0)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "norm(L.x_hat_dot): " << norm(L.x_hat_dot) << ", L.LOCALIZATION_TOLERLANCE: " << L.LOCALIZATION_TOLERLANCE);
        if (L.tol_cnt == 0)
        {
            L.tol_ts = this->get_clock()->now().seconds() - tstart_;
            // RCLCPP_INFO_STREAM(this->get_logger(), "L.tol_ts: " << L.tol_ts);
        }

        if ((L.tol_cnt + 1) == L.timer_cnt)
        {
            L.tol_cnt++;
        }
        else
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "cnt reset!!");
            L.tol_cnt = 0;
            L.timer_cnt = 0;
        }

        if (L.tol_cnt * ts_ > L.TOL_TIME)
        {
            L.tol_tf = tstart_ - this->get_clock()->now().seconds();

            RCLCPP_INFO_STREAM(this->get_logger(), "localization has completed during " << L.tol_ts << " ~ " << L.tol_tf);
            RCLCPP_INFO_STREAM(this->get_logger(), "L.tol_cnt " << L.tol_cnt * ts_ << " ~ " << L.TOL_TIME);
            return 1;
        }
    }
    return 0;
}

void FOLLOWER2_CONTROLLER::localization_pub()
{
    l_.d = L.D;
    l_.zeta1 = L.zeta1;
    l_.zeta2[0] = L.zeta2[0];
    l_.zeta2[1] = L.zeta2[1];
    l_.zeta1_dot = L.zeta1_dot;
    l_.zeta2_dot[0] = L.zeta2_dot[0];
    l_.zeta2_dot[1] = L.zeta2_dot[1];
    l_.alpha = L.alpha;
    l_.gamma = L.gamma;
    l_.z_fil = L.z_fil;
    l_.phi_fil[0] = L.phi_fil[0];
    l_.phi_fil[1] = L.phi_fil[1];
    l_.z_hat = L.z_hat;
    l_.x_hat[0] = L.x_hat[0];
    l_.x_hat[1] = L.x_hat[1];
    l_.x_hat_dot[0] = L.x_hat_dot[0];
    l_.x_hat_dot[1] = L.x_hat_dot[1];
    l_.pe_amp = L.pe_amp;
    l_.pe_freq = L.pe_freq;
    l_.timer_cnt = L.timer_cnt;
    l_.tol_cnt = L.tol_cnt;
    l_.tol_ts = L.tol_ts;
    l_.tol_tf = L.tol_tf;
    l_.tol_time = L.TOL_TIME;
    l_.localization_threshold = L.LOCALIZATION_TOLERLANCE;
}

void FOLLOWER2_CONTROLLER::leader_position_callback(const my_interfaces::msg::PolarCoor::SharedPtr msg)
{
    lpc_.header.stamp = msg->header.stamp;
    lpc_.x = msg->x;
    lpc_.y = msg->y;
    lpc_.r = msg->r;
    lpc_.theta = msg->theta;
}

void FOLLOWER2_CONTROLLER::desired_position_callback(const my_interfaces::msg::DesiredFormation::SharedPtr msg)
{
    df_.header.stamp = msg->header.stamp;
    df_.zs12_squared = msg->zs12_squared;
    df_.zs13_squared = msg->zs13_squared;
    df_.zs23_squared = msg->zs23_squared;
}

void FOLLOWER2_CONTROLLER::uwb_callback(const my_interfaces::msg::Uwb::SharedPtr msg)
{
    uwb_.header.stamp = msg->header.stamp;
    uwb_.time_stamp = msg->time_stamp;
    uwb_.range = msg->range;

    L.D = uwb_.range;
}

void FOLLOWER2_CONTROLLER::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_.header.stamp.nanosec = msg->header.stamp.nanosec;
    odom_.header.stamp.sec = msg->header.stamp.sec;

    odom_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_.pose.pose.orientation.z = msg->pose.pose.orientation.z;

    f2_pos_[0] = odom_.pose.pose.position.x;
    f2_pos_[1] = odom_.pose.pose.position.y;

    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = odom_.pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double r{}, p{}, y{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(r, p, y);
    f2_att_ = y;
}

void FOLLOWER2_CONTROLLER::leader_lost_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    leader_lost_.data = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FOLLOWER2_CONTROLLER>());
    rclcpp::shutdown();
    return 0;
}