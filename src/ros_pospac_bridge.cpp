#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>

class RosPospacBridge : public rclcpp::Node {
public:
  RosPospacBridge();

private:
  void publishGpsData();
  void publishMapToGnssTransform(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& timestamp);
  Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);
  sensor_msgs::msg::NavSatFix createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                               double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp);
  geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(double easting, double northing, double altitude,
                                                                  double roll, double pitch, double yaw,
                                                                  double east_sd, double north_sd, double height_sd,
                                                                  double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time timestamp);
  geometry_msgs::msg::PoseStamped createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance);
  geometry_msgs::msg::Pose poseWithCovarianceToPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance);
  sensor_msgs::msg::Imu createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                         double z_angular_rate, double x_acceleration, double y_acceleration,
                                         double z_acceleration, double roll, double pitch, double yaw,
                                         double roll_sd, double pitch_sd, double heading_sd);
  void publishTwistMessage(double east_velocity, double north_velocity, double up_velocity,
                           double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time);
  geometry_msgs::msg::Pose transformPoseToMapFrame(const geometry_msgs::msg::Pose& pose);

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;

  double origin_easting_;
  double origin_northing_;
  double initial_altitude_;

  std::string file_path_;

  std::vector<geometry_msgs::msg::Pose> all_poses_;  // To accumulate all poses
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // Transform broadcaster for map_to_gnss
};

RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge") {
    // Initialize transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declare parameters
    file_path_ = this->declare_parameter<std::string>("gps_data_file", "");
    RCLCPP_INFO(this->get_logger(), "GPS data file path: %s", file_path_.c_str());

    std::string mgrs_origin = this->declare_parameter<std::string>("mgrs_origin", "");
    initial_altitude_ = this->declare_parameter<double>("origin_altitude", 0.0);

    int zone;
    bool northp;
    int precision;
    bool centerp = true;

    try {
        // GeographicLib::MGRS::Reverse(mgrs_origin, zone, northp, origin_easting_, origin_northing_, precision, centerp);
        GeographicLib::MGRS::Reverse("35TPF0000000100000001", zone, northp, origin_easting_, origin_northing_, precision, centerp);
        RCLCPP_INFO(this->get_logger(), "MGRS Origin: %s converted to UTM: easting %f, northing %f", mgrs_origin.c_str(), origin_easting_, origin_northing_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert MGRS origin: %s", e.what());
        return;
    }

    // Initialize publishers
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/ros_pospac_bridge/gps_fix", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ros_pospac_bridge/imu_data", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ros_pospac_bridge/pose_with_covariance", 10);
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/ros_pospac_bridge/pose_array", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ros_pospac_bridge/twist_data", 10);
    pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ros_pospac_bridge/pose_stamped", 10);

    // Start publishing data
    publishGpsData();
}

geometry_msgs::msg::Pose RosPospacBridge::transformPoseToMapFrame(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Pose transformed_pose = pose;

    try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
        tf2::doTransform(pose, transformed_pose, transform_stamped);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
    }

    return transformed_pose;
}

void RosPospacBridge::publishMapToGnssTransform(const geometry_msgs::msg::Pose& pose, const rclcpp::Time& timestamp) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "map_to_gnss";

    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;

    transform_stamped.transform.rotation = pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
}

void RosPospacBridge::publishGpsData() {
    RCLCPP_INFO(this->get_logger(), "Attempting to open GPS data file at: %s", file_path_.c_str());

    std::ifstream file(file_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
        return;
    }

    std::string line;

    while (std::getline(file, line) && rclcpp::ok()) {
        if (line.empty()) {
            RCLCPP_WARN(this->get_logger(), "Encountered an empty line, skipping...");
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());

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
            RCLCPP_INFO(this->get_logger(), "Parsed line successfully.");

            rclcpp::Time sensor_time(static_cast<uint64_t>(time * 1e9), RCL_ROS_TIME);

            // Convert latitude and longitude to UTM
            int zone;
            bool northp;
            double utm_easting, utm_northing;
            try {
                GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error converting to UTM: %s", e.what());
                continue;
            }

            double local_easting = utm_easting - origin_easting_;
            double local_northing = utm_northing - origin_northing_;
            double relative_altitude = ortho_height - initial_altitude_;

            // Convert UTM coordinates to MGRS for output
            std::string mgrs;
            try {
                GeographicLib::MGRS::Forward(zone, northp, utm_easting, utm_northing, 5, mgrs);
                RCLCPP_INFO(this->get_logger(), "MGRS location: %s", mgrs.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error converting to MGRS: %s", e.what());
                continue;
            }

            // Continue with publishing GPS, Pose, IMU, and Twist messages
            if (gps_pub_) {
                auto gps_msg = createGpsMessage(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);
            }

            auto pose_msg = createPoseMessage(local_easting, local_northing, relative_altitude, roll, pitch, heading,
                                              east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, sensor_time);

            geometry_msgs::msg::Pose transformed_pose = transformPoseToMapFrame(pose_msg.pose.pose);

            if (pose_pub_) {
                pose_msg.pose.pose = transformed_pose;
                pose_pub_->publish(pose_msg);
            }

            if (pose_stamped_pub_) {
                auto pose_stamped_msg = createPoseStampedMessage(pose_msg);
                pose_stamped_pub_->publish(pose_stamped_msg);
            }

            all_poses_.push_back(transformed_pose);

            if (pose_array_pub_) {
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = pose_msg.header.stamp;
                pose_array_msg.header.frame_id = "map";  // Ensure this is correct
                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);

                // Publish the transform from map to the last pose (GNSS)
                publishMapToGnssTransform(transformed_pose, pose_msg.header.stamp);
            }

            if (imu_pub_) {
                auto imu_msg = createImuMessage(sensor_time, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);
            }

            if (twist_pub_) {
                publishTwistMessage(east_velocity, north_velocity, up_velocity,
                                    x_angular_rate, y_angular_rate, z_angular_rate, sensor_time);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse line: %s", line.c_str());
        }

        rclcpp::spin_some(this->get_node_base_interface());
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

geometry_msgs::msg::PoseWithCovarianceStamped RosPospacBridge::createPoseMessage(double easting, double northing, double altitude,
                                                                double roll, double pitch, double yaw,
                                                                double east_sd, double north_sd, double height_sd,
                                                                double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time sensor_time_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp  = sensor_time_;
    pose_msg.header.frame_id = "map";  // Ensure consistent frame ID

    // Set position
    pose_msg.pose.pose.position.x = easting;
    pose_msg.pose.pose.position.y = northing;
    pose_msg.pose.pose.position.z = altitude;

    // Convert from Euler angles (assuming they are in degrees) to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // Set covariance (using the variances calculated from the standard deviations)
    pose_msg.pose.covariance[0] = east_sd * east_sd;    // Variance in X (easting)
    pose_msg.pose.covariance[7] = north_sd * north_sd;  // Variance in Y (northing)
    pose_msg.pose.covariance[14] = height_sd * height_sd;  // Variance in Z (altitude)

    pose_msg.pose.covariance[21] = roll_sd * roll_sd;   // Variance in roll
    pose_msg.pose.covariance[28] = pitch_sd * pitch_sd; // Variance in pitch
    pose_msg.pose.covariance[35] = yaw_sd * yaw_sd;     // Variance in yaw

    return pose_msg;
}

geometry_msgs::msg::Pose RosPospacBridge::poseWithCovarianceToPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance) {
    geometry_msgs::msg::Pose pose;
    pose.position = pose_with_covariance.pose.pose.position;
    pose.orientation = pose_with_covariance.pose.pose.orientation;
    return pose;
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

geometry_msgs::msg::PoseStamped RosPospacBridge::createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = pose_with_covariance.header;
    pose_stamped_msg.pose = pose_with_covariance.pose.pose;
    pose_stamped_msg.header.frame_id = "map";  // Ensure consistent frame ID
    return pose_stamped_msg;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosPospacBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
