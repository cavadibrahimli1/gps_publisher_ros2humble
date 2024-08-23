#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
  void broadcastTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);
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

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double origin_easting_;
  double origin_northing_;
  double initial_altitude_;

  std::string file_path_;

  std::vector<geometry_msgs::msg::Pose> all_poses_;  // To accumulate all poses

  struct Transform {
    double x, y, z, roll, pitch, yaw;
  } lidar_to_gnss_transform_, lidar_to_base_link_transform_;
};

RosPospacBridge::RosPospacBridge() : Node("ros_pospac_bridge") {

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Declare parameters
  bool start_gps_publisher = this->declare_parameter<bool>("start_gps_publisher", true);
  bool start_imu_publisher = this->declare_parameter<bool>("start_imu_publisher", true);
  bool start_pose_publisher = this->declare_parameter<bool>("start_pose_publisher", true);
  bool start_pose_array_publisher = this->declare_parameter<bool>("start_pose_array_publisher", true);
  bool start_twist_publisher = this->declare_parameter<bool>("start_twist_publisher", true);
  bool start_pose_stamped_publisher = this->declare_parameter<bool>("start_pose_stamped_publisher", true);

  // Initialize publishers based on parameters
  if (start_gps_publisher) {
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/ros_pospac_bridge/gps_fix", 10);
  }
  if (start_imu_publisher) {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ros_pospac_bridge/imu_data", 10);
  }
  if (start_pose_publisher) {
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ros_pospac_bridge/pose_with_covariance", 10);
  }
  if (start_pose_array_publisher) {
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/ros_pospac_bridge/pose_array", 10);
  }
  if (start_twist_publisher) {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ros_pospac_bridge/twist_data", 10);
  }
  if (start_pose_stamped_publisher) {
    pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ros_pospac_bridge/pose_stamped", 10);
  }

  // Declare other parameters
  file_path_ = this->declare_parameter<std::string>("gps_data_file", "");

  // Convert Istanbul's latitude and longitude to MGRS
  double istanbul_latitude = 41.0082;
  double istanbul_longitude = 28.9784;

  int istanbul_zone;
  bool istanbul_northp;
  double istanbul_easting, istanbul_northing;

  try {
    GeographicLib::UTMUPS::Forward(istanbul_latitude, istanbul_longitude, istanbul_zone, istanbul_northp, istanbul_easting, istanbul_northing);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error converting Istanbul's coordinates to UTM: %s", e.what());
  }

  std::string istanbul_mgrs;
  try {
    GeographicLib::MGRS::Forward(istanbul_zone, istanbul_northp, istanbul_easting, istanbul_northing, 5, istanbul_mgrs);
    RCLCPP_INFO(this->get_logger(), "Istanbul's MGRS Coordinates: %s", istanbul_mgrs.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error converting to MGRS: %s", e.what());
  }

  // Use Istanbul's MGRS coordinates as the origin
  int origin_zone_;
  bool origin_northp_;
  int prec;
  bool centerp = true;
  try {
    GeographicLib::MGRS::Reverse(istanbul_mgrs, origin_zone_, origin_northp_, origin_easting_, origin_northing_, prec, centerp);
    RCLCPP_INFO(this->get_logger(), "Origin set to Istanbul's MGRS location.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error reversing Istanbul's MGRS coordinates: %s", e.what());
  }

  initial_altitude_ = this->declare_parameter<double>("origin_altitude", 0.0);

  // Start publishing data
  publishGpsData();
}

void RosPospacBridge::broadcastTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg) {
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";

  transformStamped.transform.translation.x = pose_msg.pose.pose.position.x;
  transformStamped.transform.translation.y = pose_msg.pose.pose.position.y;
  transformStamped.transform.translation.z = pose_msg.pose.pose.position.z;

  transformStamped.transform.rotation = pose_msg.pose.pose.orientation;

  // Send the transformation
  tf_broadcaster_->sendTransform(transformStamped);
}

void RosPospacBridge::publishGpsData() {
    std::ifstream file(file_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
        return;
    }

    std::string line;

    while (std::getline(file, line) && rclcpp::ok()) {
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

            // Convert latitude and longitude to MGRS
            int zone;
            bool northp;
            double utm_easting, utm_northing;
            try {
                GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error converting to UTM: %s", e.what());
                continue;
            }

            // Convert UTM coordinates to MGRS
            std::string mgrs;
            try {
                GeographicLib::MGRS::Forward(zone, northp, utm_easting, utm_northing, 5, mgrs);
                RCLCPP_INFO(this->get_logger(), "MGRS: %s", mgrs.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error converting to MGRS: %s", e.what());
                continue;
            }

            // Parse the MGRS coordinates back into UTM to simulate position in easting/northing
            double local_easting, local_northing;
            int prec;
            bool centerp = true;
            try {
                GeographicLib::MGRS::Reverse(mgrs, zone, northp, local_easting, local_northing, prec, centerp);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error reversing MGRS coordinates: %s", e.what());
                continue;
            }

            double relative_altitude = ortho_height - initial_altitude_;

            // Continue with publishing GPS, Pose, IMU, and Twist messages using MGRS-based easting/northing
            if (gps_pub_) {
                auto gps_msg = createGpsMessage(latitude, longitude, ellipsoid_height,
                                                east_sd, north_sd, height_sd, sensor_time);
                gps_pub_->publish(gps_msg);  // Publish GPS Fix message
            }

            auto pose_msg = createPoseMessage(local_easting, local_northing, relative_altitude, roll, pitch, heading,
                                              east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, sensor_time);

            if (pose_pub_) {
                pose_pub_->publish(pose_msg);
            }

            if (pose_stamped_pub_) {
                auto pose_stamped_msg = createPoseStampedMessage(pose_msg);
                pose_stamped_pub_->publish(pose_stamped_msg);
            }

            all_poses_.push_back(poseWithCovarianceToPose(pose_msg));

            if (pose_array_pub_) {
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = pose_msg.header.stamp;
                pose_array_msg.header.frame_id = "base_link";

                for (auto &pose : all_poses_) {
                    try {
                        // Convert UTM to MGRS and back again for publishing (already handled before in the loop)
                        GeographicLib::MGRS::Forward(zone, northp, pose.position.x, pose.position.y, 5, mgrs);
                        GeographicLib::MGRS::Reverse(mgrs, zone, northp, pose.position.x, pose.position.y, prec, centerp);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Error handling MGRS for PoseArray: %s", e.what());
                        continue;
                    }
                }

                pose_array_msg.poses = all_poses_;
                pose_array_pub_->publish(pose_array_msg);
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

            // Broadcast the transformation
            broadcastTransform(pose_msg);
        }

        rclcpp::spin_some(this->get_node_base_interface());
    }
    file.close();
}


sensor_msgs::msg::NavSatFix RosPospacBridge::createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                             double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp) {
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = timestamp;
    gps_msg.header.frame_id = "base_link";
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
    pose_msg.header.frame_id = "base_link";

    // Set position
    pose_msg.pose.pose.position.x = easting;
    pose_msg.pose.pose.position.y = northing;
    pose_msg.pose.pose.position.z = altitude;

    // Convert from Euler angles to quaternion
    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    // Set covariance
    pose_msg.pose.covariance[0] = east_sd * east_sd;
    pose_msg.pose.covariance[7] = north_sd * north_sd;
    pose_msg.pose.covariance[14] = height_sd * height_sd;

    pose_msg.pose.covariance[21] = roll_sd * roll_sd;
    pose_msg.pose.covariance[28] = pitch_sd * pitch_sd;
    pose_msg.pose.covariance[35] = yaw_sd * yaw_sd;

    return pose_msg;
}

geometry_msgs::msg::PoseStamped RosPospacBridge::createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = pose_with_covariance.header;
    pose_stamped_msg.pose = pose_with_covariance.pose.pose;
    return pose_stamped_msg;
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
    (void)roll_sd;  // Unused parameter
    (void)pitch_sd;  // Unused parameter
    (void)heading_sd;  // Unused parameter

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = timestamp;
    imu_msg.header.frame_id = "base_link";

    Eigen::Quaterniond q = getQuaternionFromRPY(roll, pitch, yaw);
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
    twist_with_cov_stamped_msg.header.frame_id = "base_link";

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

Eigen::Quaterniond RosPospacBridge::getQuaternionFromRPY(double roll, double pitch, double yaw) {
    double roll_in_rad = roll * M_PI / 180.0;
    double pitch_in_rad = pitch * M_PI / 180.0;
    double yaw_in_rad = yaw * M_PI / 180.0;

    // Convert RPY angles to quaternion
    Eigen::AngleAxisd roll_angle(roll_in_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch_in_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw_in_rad, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    return q;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosPospacBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
