#include "ros_pospac_bridge/ros_pospac_bridge.hpp"

RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge") {
    // Initialize transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    file_path_ = this->declare_parameter<std::string>("gps_data_file", "");
    initial_altitude_ = this->declare_parameter<double>("origin_altitude", 0.0);

    std::string mgrs_origin = this->declare_parameter<std::string>("mgrs_origin", "");
    
    int zone;
    bool northp;
    int precision;
    bool centerp = true;
    try {
        GeographicLib::MGRS::Reverse(mgrs_origin, zone, northp, origin_easting_, origin_northing_, precision, centerp);
    } catch (const std::exception& e) {
        return;
    }

    enable_gps_pub_ = this->declare_parameter<bool>("publishers.nav_sat_fix.enable", true);
    if (enable_gps_pub_) {
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/ros_pospac_bridge/gps_fix", 10);
    }

    enable_imu_pub_ = this->declare_parameter<bool>("publishers.imu.enable", true);
    if (enable_imu_pub_) {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ros_pospac_bridge/imu_data", 10);
    }

    enable_pose_pub_ = this->declare_parameter<bool>("publishers.pose_with_covariance_stamped.enable", true);
    if (enable_pose_pub_) {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ros_pospac_bridge/pose_with_covariance", 10);
    }

    enable_pose_array_pub_ = this->declare_parameter<bool>("publishers.pose_array.enable", true);
    if (enable_pose_array_pub_) {
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/ros_pospac_bridge/pose_array", 10);
    }

    enable_twist_pub_ = this->declare_parameter<bool>("publishers.twist_with_covariance_stamped.enable", true);
    if (enable_twist_pub_) {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ros_pospac_bridge/twist_with_covariance", 10);
    }

    enable_pose_stamped_pub_ = this->declare_parameter<bool>("publishers.pose_stamped.enable", true);
    if (enable_pose_stamped_pub_) {
        pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ros_pospac_bridge/pose_stamped", 10);
    }

    enable_tf_pub_ = this->declare_parameter<bool>("publishers.tf.enable", true);
    if (enable_tf_pub_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    // Initialize the transformation between lidar and gnss_ins
    lidar_to_gnss_transform_ = {
        0.0,    // x
        0.0,    // y
        -0.08,  // z (sensor is below GNSS)
        0.007237,  // roll
        -0.0072323,  // pitch
        3.1334945  // yaw
    };

    // Start publishing data
    publishGpsData();
}

// Function to initialize base_link based on GNSS pose
geometry_msgs::msg::Pose RosPospacBridge::initializeBaseLinkPose(const geometry_msgs::msg::Pose& gnss_ins_pose) {
    geometry_msgs::msg::Pose base_link_pose = gnss_ins_pose;

    // Apply GNSS to base_link transform
    tf2::Quaternion q_orig, q_rot, q_final;
    tf2::fromMsg(gnss_ins_pose.orientation, q_orig);

    // Rotation transformation from GNSS to base_link
    // Adjust yaw by adding PI (180 degrees) to rotate the base link orientation
    q_rot.setRPY(lidar_to_gnss_transform_.roll, lidar_to_gnss_transform_.pitch, lidar_to_gnss_transform_.yaw + M_PI);
    q_final = q_rot * q_orig;
    q_final.normalize();

    base_link_pose.orientation = tf2::toMsg(q_final);
    base_link_pose.position.x += lidar_to_gnss_transform_.x;
    base_link_pose.position.y += lidar_to_gnss_transform_.y;
    base_link_pose.position.z += lidar_to_gnss_transform_.z;

    return base_link_pose;
}

// Publish transform: map -> base_link (with base_link initialized based on GNSS pose)
void RosPospacBridge::publishMapToBaseLinkTransform(const geometry_msgs::msg::Pose& gnss_ins_pose, const rclcpp::Time& timestamp) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    geometry_msgs::msg::Pose base_link_pose = initializeBaseLinkPose(gnss_ins_pose);

    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = "map";  // Parent frame (map)
    transform_stamped.child_frame_id = "base_link";  // Child frame (robot base)

    // Set translation (base_link's global position based on GNSS pose)
    transform_stamped.transform.translation.x = base_link_pose.position.x;
    transform_stamped.transform.translation.y = base_link_pose.position.y;
    transform_stamped.transform.translation.z = base_link_pose.position.z;

    // Set rotation (base_link's global orientation)
    transform_stamped.transform.rotation = base_link_pose.orientation;

    // Publish the transform
    tf_broadcaster_->sendTransform(transform_stamped);
}

void RosPospacBridge::publishGpsData() {
    std::ifstream file(file_path_);
    if (!file.is_open()) {
        return;  // Silently handle file open failure
    }

    std::string line;

    while (std::getline(file, line) && rclcpp::ok()) {
        if (line.empty()) {
            continue;  // Skip empty lines silently
        }

        std::istringstream iss(line);
        double time, distance, easting, northing, ortho_height, latitude, longitude, ellipsoid_height;
        double roll, pitch, heading, east_velocity, north_velocity, up_velocity;
        double x_angular_rate, y_angular_rate, z_angular_rate;
        double x_acceleration, y_acceleration, z_acceleration;
        double east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd;

        if (iss >> time >> distance >> easting >> northing >> ortho_height >> latitude >> longitude >> ellipsoid_height
               >> roll >> pitch >> heading >> east_velocity >> north_velocity >> up_velocity
               >> x_angular_rate >> y_angular_rate >> z_angular_rate
               >> x_acceleration >> y_acceleration >> z_acceleration
               >> east_sd >> north_sd >> height_sd >> roll_sd >> pitch_sd >> heading_sd) {

            rclcpp::Time sensor_time(static_cast<uint64_t>(time * 1e9), RCL_ROS_TIME);

            int zone;
            bool northp;
            double utm_easting, utm_northing;
            GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);

            double relative_altitude = ortho_height;  // Use actual ortho_height directly

            std::string mgrs;
            GeographicLib::MGRS::Forward(zone, northp, utm_easting, utm_northing, 8, mgrs);

            double mgrs_x = std::stod(mgrs.substr(5, 8))/1000;
            double mgrs_y = std::stod(mgrs.substr(13, 8))/1000;

            // RCLCPP_INFO(this->get_logger(), "MGRS location: %s, Relative X: %f, Relative Y: %f", mgrs.c_str(), mgrs_x, mgrs_y);

            // Continue with publishing GPS, Pose, IMU, and Twist messages
            if (enable_gps_pub_ && gps_pub_) {
                auto gps_msg = createGpsMessage(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);
            }

            auto pose_msg = createPoseMessage(latitude, longitude, relative_altitude, roll, pitch, heading,
                                              east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, sensor_time, mgrs);

            geometry_msgs::msg::Pose base_link_pose = pose_msg.pose.pose;

            if (enable_pose_pub_ && pose_pub_) {
                pose_pub_->publish(pose_msg);
            }

            if (enable_pose_stamped_pub_ && pose_stamped_pub_) {
                auto pose_stamped_msg = createPoseStampedMessage(pose_msg);
                pose_stamped_pub_->publish(pose_stamped_msg);
            }

            all_poses_.push_back(base_link_pose);

            if (enable_pose_array_pub_ && pose_array_pub_) {
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = pose_msg.header.stamp;
                pose_array_msg.header.frame_id = "map";
                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);
            }

            // Publish map -> base_link (initialized using GNSS) transform
            if (enable_tf_pub_) {
                publishMapToBaseLinkTransform(base_link_pose, sensor_time);
            }

            if (enable_imu_pub_ && imu_pub_) {
                auto imu_msg = createImuMessage(sensor_time, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);
            }

            if (enable_twist_pub_ && twist_pub_) {
                publishTwistMessage(east_velocity, north_velocity, up_velocity,
                                    x_angular_rate, y_angular_rate, z_angular_rate, sensor_time);
            }
        }
    }
    file.close();
}

Eigen::Quaterniond RosPospacBridge::getQuaternionFromRPY(double roll, double pitch, double yaw) {
    double roll_in_rad = roll * M_PI / 180.0;
    double pitch_in_rad = pitch * M_PI / 180.0;
    double yaw_in_rad = yaw * M_PI / 180.0;

    // Convert ENU to NED
    Eigen::AngleAxisd angle_axis_x(roll_in_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_axis_y(pitch_in_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_axis_z(-yaw_in_rad + 1.5708, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d orientation_enu(angle_axis_z * angle_axis_y * angle_axis_x);

    Eigen::Quaterniond q(orientation_enu);

    return q;
}

sensor_msgs::msg::NavSatFix RosPospacBridge::createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                             double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp) {
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = timestamp;
    gps_msg.header.frame_id = "map";  // Ensure consistent frame ID
    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gps_msg.latitude = latitude;
    gps_msg.longitude = longitude;
    gps_msg.altitude = ellipsoid_height;

    gps_msg.position_covariance[0] = east_sd * east_sd;
    gps_msg.position_covariance[4] = north_sd * north_sd;
    gps_msg.position_covariance[8] = height_sd * height_sd;

    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    return gps_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped RosPospacBridge::createPoseMessage(
    double latitude, double longitude, double altitude,
    double roll, double pitch, double yaw,
    double east_sd, double north_sd, double height_sd,
    double roll_sd, double pitch_sd, double yaw_sd, 
    rclcpp::Time sensor_time, const std::string& /* mgrs */) {  // Add comment to indicate unused parameter

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = sensor_time;
    pose_msg.header.frame_id = "map";  // Make sure this matches your map frame

    // Convert latitude and longitude to MGRS
    int zone;
    bool northp;
    double utm_easting, utm_northing;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);

    std::string mgrs;
    GeographicLib::MGRS::Forward(zone, northp, utm_easting, utm_northing, 8, mgrs);

    double mgrs_x = std::stod(mgrs.substr(5, 8)) / 1000;
    double mgrs_y = std::stod(mgrs.substr(13, 8)) / 1000;

    // Use the MGRS-relative coordinates for the pose position
    pose_msg.pose.pose.position.x = mgrs_x;  // Relative X
    pose_msg.pose.pose.position.y = mgrs_y;  // Relative Y
    pose_msg.pose.pose.position.z = altitude;  // Use actual altitude directly

    // Convert from Euler angles (degrees) to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // Set covariance (based on standard deviations)
    pose_msg.pose.covariance[0] = east_sd * east_sd;    // Variance in X (easting)
    pose_msg.pose.covariance[7] = north_sd * north_sd;  // Variance in Y (northing)
    pose_msg.pose.covariance[14] = height_sd * height_sd;  // Variance in Z (altitude)
    pose_msg.pose.covariance[21] = roll_sd * roll_sd;   // Variance in roll
    pose_msg.pose.covariance[28] = pitch_sd * pitch_sd; // Variance in pitch
    pose_msg.pose.covariance[35] = yaw_sd * yaw_sd;     // Variance in yaw

    // Transform pose to base_link frame
    pose_msg.pose.pose = transformPoseToBaseLink(pose_msg.pose.pose);

    return pose_msg;
}

geometry_msgs::msg::Pose RosPospacBridge::transformPoseToBaseLink(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Pose base_link_pose = pose;

    // Apply GNSS to base_link transform
    tf2::Quaternion q_orig, q_rot, q_final;
    tf2::fromMsg(pose.orientation, q_orig);

    // Rotation transformation from GNSS to base_link
    q_rot.setRPY(lidar_to_gnss_transform_.roll, lidar_to_gnss_transform_.pitch, lidar_to_gnss_transform_.yaw + M_PI);
    q_final = q_rot * q_orig;
    q_final.normalize();

    base_link_pose.orientation = tf2::toMsg(q_final);
    base_link_pose.position.x += lidar_to_gnss_transform_.x;
    base_link_pose.position.y += lidar_to_gnss_transform_.y;
    base_link_pose.position.z += lidar_to_gnss_transform_.z;

    return base_link_pose;
}

geometry_msgs::msg::PoseStamped RosPospacBridge::createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = pose_with_covariance.header;
    pose_stamped_msg.pose = pose_with_covariance.pose.pose;
    pose_stamped_msg.header.frame_id = "map";  // Ensure consistent frame ID
    return pose_stamped_msg;
}

sensor_msgs::msg::Imu RosPospacBridge::createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                                        double z_angular_rate, double x_acceleration, double y_acceleration,
                                                        double z_acceleration, double roll, double pitch, double yaw,
                                                        double /* roll_sd */, double /* pitch_sd */, double /* heading_sd */) {
    // Create the IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = "map";  // Ensure consistent frame ID

    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);  // Assuming angles are in degrees
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    imu_msg.angular_velocity.x = x_angular_rate;
    imu_msg.angular_velocity.y = y_angular_rate;
    imu_msg.angular_velocity.z = z_angular_rate;

    imu_msg.linear_acceleration.x = x_acceleration;
    imu_msg.linear_acceleration.y = y_acceleration;
    imu_msg.linear_acceleration.z = z_acceleration;

    imu_msg.orientation_covariance[0] = 1.0;
    imu_msg.orientation_covariance[4] = 1.0;
    imu_msg.orientation_covariance[8] = 1.0;

    imu_msg.angular_velocity_covariance[0] = 1.0;
    imu_msg.angular_velocity_covariance[4] = 1.0;
    imu_msg.angular_velocity_covariance[8] = 1.0;

    imu_msg.linear_acceleration_covariance[0] = 1.0;
    imu_msg.linear_acceleration_covariance[4] = 1.0;
    imu_msg.linear_acceleration_covariance[8] = 1.0;

    return imu_msg;
}

void RosPospacBridge::publishTwistMessage(double east_velocity, double north_velocity, double up_velocity,
                                          double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_stamped_msg;

    twist_with_cov_stamped_msg.header.stamp = sensor_time;
    twist_with_cov_stamped_msg.header.frame_id = "map";  // Ensure consistent frame ID

    // Set linear velocities
    twist_with_cov_stamped_msg.twist.twist.linear.x = east_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.y = north_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.z = up_velocity;

    // Set angular velocities
    twist_with_cov_stamped_msg.twist.twist.angular.x = x_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.y = y_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.z = z_angular_rate;

    twist_with_cov_stamped_msg.twist.covariance[0] = 1.0;  // Variance in X (east velocity)
    twist_with_cov_stamped_msg.twist.covariance[7] = 1.0;  // Variance in Y (north velocity)
    twist_with_cov_stamped_msg.twist.covariance[14] = 1.0;  // Variance in Z (up velocity)

    // Publish the Twist message
    twist_pub_->publish(twist_with_cov_stamped_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosPospacBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}