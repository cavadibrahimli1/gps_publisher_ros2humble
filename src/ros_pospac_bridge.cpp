// ROS2 Humble 
// ROS2 Version: 0.14.0

// This code is responsible for bridging GPS data
// from a file to various ROS2 topics. It includes functionalities for publishing
// GPS, IMU, pose, twist, and transform data, as well as handling calibration parameters.

#include "ros_pospac_bridge/ros_pospac_bridge.hpp"

RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge") {

    file_path_ = this->declare_parameter<std::string>("gps_data_file", "");
    bool enable_gps_pub_ = this->declare_parameter<bool>("publishers.nav_sat_fix.enable", true);
    bool enable_imu_pub_ = this->declare_parameter<bool>("publishers.imu.enable", true);
    bool enable_pose_with_cov_stamped_pub_ = this->declare_parameter<bool>("publishers.pose_with_covariance_stamped.enable", true);
    bool enable_pose_array_pub_ = this->declare_parameter<bool>("publishers.pose_array.enable", true);
    bool enable_twist_pub_ = this->declare_parameter<bool>("publishers.twist_with_covariance_stamped.enable", true);
    bool enable_pose_stamped_pub_ = this->declare_parameter<bool>("publishers.pose_stamped.enable", true);
    bool enable_tf_pub_ = this->declare_parameter<bool>("publishers.tf.enable", true);
    bool enable_gnss_ins_orientation_pub_ = this->declare_parameter<bool>("publishers.gnss_ins_orientation.enable", true);

    if (enable_gps_pub_) {
        std::string gps_topic = this->declare_parameter<std::string>("publishers.nav_sat_fix.topic", "/ros_pospac_bridge/gps_fix");
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic, 10);
    }
    if (enable_imu_pub_) {
        std::string imu_topic = this->declare_parameter<std::string>("publishers.imu.topic", "/ros_pospac_bridge/imu_data");
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    }
    if (enable_pose_with_cov_stamped_pub_) {
        std::string pose_with_covariance_stamped_topic = this->declare_parameter<std::string>("publishers.pose_with_covariance_stamped.topic", "/ros_pospac_bridge/pose_with_covariance");
        pose_with_cov_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_with_covariance_stamped_topic, 10);
    }
    if (enable_pose_array_pub_) {
        std::string pose_array_topic = this->declare_parameter<std::string>("publishers.pose_array.topic", "/ros_pospac_bridge/pose_array");
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(pose_array_topic, 10);
    }
    if (enable_twist_pub_) {
        std::string twist_topic = this->declare_parameter<std::string>("publishers.twist_with_covariance_stamped.topic", "/ros_pospac_bridge/twist_with_covariance");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic, 10);
    }
    if (enable_pose_stamped_pub_) {
        std::string pose_stamped_topic = this->declare_parameter<std::string>("publishers.pose_stamped.topic", "/ros_pospac_bridge/pose_stamped");
        pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_stamped_topic, 10);
    }
    if (enable_tf_pub_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    if (enable_gnss_ins_orientation_pub_) {
        std::string gnss_ins_orientation_topic = this->declare_parameter<std::string>("publishers.gnss_ins_orientation.topic", "/ros_pospac_bridge/gnss_ins_orientation");
        gnss_ins_orientation_pub_ = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(gnss_ins_orientation_topic, 10);
    }

    getCalibrations();
    CreatePublishData();
}

void RosPospacBridge::getCalibrations() {
    tf2::Transform lidar_to_gnss_transform;
    lidar_to_gnss_transform.setOrigin(tf2::Vector3(
        this->declare_parameter<double>("calibration.lidar_to_gnss.x", 0.0),
        this->declare_parameter<double>("calibration.lidar_to_gnss.y", 0.0),
        this->declare_parameter<double>("calibration.lidar_to_gnss.z", 0.0)
    ));
    tf2::Quaternion q_rot;
    q_rot.setRPY(
        this->declare_parameter<double>("calibration.lidar_to_gnss.roll", 0.0),
        this->declare_parameter<double>("calibration.lidar_to_gnss.pitch", 0.0),
        this->declare_parameter<double>("calibration.lidar_to_gnss.yaw", 0.0)
    );
    lidar_to_gnss_transform.setRotation(q_rot);
    lidar_to_gnss_transform_ = lidar_to_gnss_transform;

    // Read base_link_to_lidar parameters
    tf2::Transform base_link_to_lidar_transform;
    base_link_to_lidar_transform.setOrigin(tf2::Vector3(
        this->declare_parameter<double>("calibration.base_link_to_lidar.x", 0.0),
        this->declare_parameter<double>("calibration.base_link_to_lidar.y", 0.0),
        this->declare_parameter<double>("calibration.base_link_to_lidar.z", 0.0)
    ));
    tf2::Quaternion q_base_link_to_lidar;
    q_base_link_to_lidar.setRPY(
        this->declare_parameter<double>("calibration.base_link_to_lidar.roll", 0.0),
        this->declare_parameter<double>("calibration.base_link_to_lidar.pitch", 0.0),
        this->declare_parameter<double>("calibration.base_link_to_lidar.yaw", 0.0)
    );
    base_link_to_lidar_transform.setRotation(q_base_link_to_lidar);
    base_link_to_lidar_transform_ = base_link_to_lidar_transform;
}

geometry_msgs::msg::Pose RosPospacBridge::transformPoseToBaseLink(const geometry_msgs::msg::Pose& gnss_pose) {
    tf2::Transform tf_map2gnss_transform;
    tf2::fromMsg(gnss_pose, tf_map2gnss_transform);

    // Apply lidar_to_gnss_transform and then base_link_to_lidar_transform
    tf2::Transform tf_map2base_link = tf_map2gnss_transform * lidar_to_gnss_transform_.inverse() * base_link_to_lidar_transform_.inverse();

    geometry_msgs::msg::Pose pose_in_base_link;
    tf2::toMsg(tf_map2base_link, pose_in_base_link);

    return pose_in_base_link;
}

void RosPospacBridge::publishMapToBaseLinkTransform(const geometry_msgs::msg::Pose& base_link_pose, const rclcpp::Time& timestamp) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = base_link_pose.position.x;
    transform_stamped.transform.translation.y = base_link_pose.position.y;
    transform_stamped.transform.translation.z = base_link_pose.position.z;
    transform_stamped.transform.rotation = base_link_pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
}

void RosPospacBridge::CreatePublishData() {
    std::ifstream file(file_path_);
    if (!file.is_open()) {
        return; 
    }

    std::string line;
    rclcpp::Time start_time = this->now();
    double first_timestamp = 0.0;
    bool first_line = true;

    while (std::getline(file, line) && rclcpp::ok()) {
        if (line.empty()) {
            continue; 
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

            if (first_line) {
                first_timestamp = time;
                first_line = false;
            }

            rclcpp::Time sensor_time(static_cast<uint64_t>(time * 1e9), RCL_ROS_TIME);
            rclcpp::Time current_time = this->now();
            double elapsed_time = (time - first_timestamp);
            double real_elapsed_time = (current_time - start_time).seconds();

            if (elapsed_time > real_elapsed_time) {
                std::this_thread::sleep_for(std::chrono::duration<double>(elapsed_time - real_elapsed_time));
            }

            // Create pose message with only needed values
            geometry_msgs::msg::Pose base_link_pose = createPoseMessage(latitude, longitude, ellipsoid_height, roll, pitch, heading);

            // Create and publish pose stamped message
            if (pose_stamped_pub_) {
                geometry_msgs::msg::PoseStamped pose_stamped_msg;
                pose_stamped_msg.header.stamp = sensor_time;
                pose_stamped_msg.header.frame_id = "map";
                pose_stamped_msg.pose = base_link_pose;
                pose_stamped_pub_->publish(pose_stamped_msg);
            }

            // Publish pose message
            if (pose_with_cov_stamped_pub_) {
                geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg;
                pose_with_cov_msg.header.stamp = sensor_time;
                pose_with_cov_msg.header.frame_id = "map";
                pose_with_cov_msg.pose.pose = base_link_pose;
                pose_with_cov_msg.pose.covariance[0] = east_sd * east_sd;
                pose_with_cov_msg.pose.covariance[7] = north_sd * north_sd;
                pose_with_cov_msg.pose.covariance[14] = height_sd * height_sd;
                pose_with_cov_msg.pose.covariance[21] = roll_sd * roll_sd;
                pose_with_cov_msg.pose.covariance[28] = pitch_sd * pitch_sd;
                pose_with_cov_msg.pose.covariance[35] = heading_sd * heading_sd;
                pose_with_cov_stamped_pub_->publish(pose_with_cov_msg);
            }
            // Create and publish pose array message
            if (pose_array_pub_) {
                all_poses_.push_back(base_link_pose);
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = sensor_time;
                pose_array_msg.header.frame_id = "map";
                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);
            }

            // Publish map -> base_link (initialized using GNSS) transform
            if (tf_broadcaster_) {
                publishMapToBaseLinkTransform(base_link_pose, sensor_time);
            }

            // Create and publish IMU message
            if (imu_pub_) {
                auto imu_msg = createImuMessage(sensor_time, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);
            }

            // Create and publish twist message
            if (twist_pub_) {
                publishTwistMessage(east_velocity, north_velocity, up_velocity,
                                    x_angular_rate, y_angular_rate, z_angular_rate, sensor_time);
            }

            // Create and publish GPS message
            if (gps_pub_) {
                auto gps_msg = createGpsMessage(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);
            }

            if (gnss_ins_orientation_pub_) {
                autoware_sensing_msgs::msg::GnssInsOrientationStamped gnss_ins_orientation_msg;
                gnss_ins_orientation_msg.header.stamp = sensor_time;
                gnss_ins_orientation_msg.header.frame_id = "gnss_ins_link";
                gnss_ins_orientation_msg.orientation.orientation = base_link_pose.orientation;
                gnss_ins_orientation_msg.orientation.rmse_rotation_x = roll_sd;
                gnss_ins_orientation_msg.orientation.rmse_rotation_y = pitch_sd;
                gnss_ins_orientation_msg.orientation.rmse_rotation_z = heading_sd;
                gnss_ins_orientation_pub_->publish(gnss_ins_orientation_msg);
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
    gps_msg.header.frame_id = "gnss_ins_link";
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

geometry_msgs::msg::Pose RosPospacBridge::createPoseMessage(
    double latitude, double longitude, double altitude,
    double roll, double pitch, double yaw) {

    geometry_msgs::msg::Pose pose_msg;

    // Convert latitude and longitude to MGRS
    int zone;
    bool northp;
    double utm_easting, utm_northing;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);

    std::string mgrs;
    GeographicLib::MGRS::Forward(zone, northp, utm_easting, utm_northing, 8, mgrs);

    double mgrs_x = std::stod(mgrs.substr(5, 8)) / 1000; 
    double mgrs_y = std::stod(mgrs.substr(13, 8)) / 1000;

    pose_msg.position.x = mgrs_x;  // Relative X
    pose_msg.position.y = mgrs_y;  // Relative Y
    pose_msg.position.z = altitude;  // Use actual altitude directly

    // Convert from Euler angles (degrees) to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();


    // Transform pose to base_link frame
    pose_msg = transformPoseToBaseLink(pose_msg);

    return pose_msg;
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
                                                        double roll_sd, double pitch_sd, double heading_sd) {
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

    // Set orientation covariance using roll_sd, pitch_sd, and heading_sd
    imu_msg.orientation_covariance[0] = roll_sd * roll_sd;
    imu_msg.orientation_covariance[4] = pitch_sd * pitch_sd;
    imu_msg.orientation_covariance[8] = heading_sd * heading_sd;

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
    twist_with_cov_stamped_msg.header.frame_id = "map";  // FRAME ID CHANGE

    // Set linear velocities
    twist_with_cov_stamped_msg.twist.twist.linear.x = east_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.y = north_velocity;
    twist_with_cov_stamped_msg.twist.twist.linear.z = up_velocity;

    // Set angular velocities
    twist_with_cov_stamped_msg.twist.twist.angular.x = x_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.y = y_angular_rate;
    twist_with_cov_stamped_msg.twist.twist.angular.z = z_angular_rate;

    twist_with_cov_stamped_msg.twist.covariance[0] = 1.0; 
    twist_with_cov_stamped_msg.twist.covariance[7] = 1.0;  
    twist_with_cov_stamped_msg.twist.covariance[14] = 1.0;  

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
