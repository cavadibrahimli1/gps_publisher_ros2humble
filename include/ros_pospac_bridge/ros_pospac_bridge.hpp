#ifndef ROS_POSPAC_BRIDGE_ROS_POSPAC_BRIDGE_HPP_
#define ROS_POSPAC_BRIDGE_ROS_POSPAC_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // Include PoseStamped message
#include <Eigen/Geometry>

class RosPospacBridge : public rclcpp::Node {
public:
  RosPospacBridge();
  
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;  // Declare PoseStamped publisher

private:
  double origin_easting_;
  double origin_northing_;
  double initial_altitude_;

  std::string file_path_;

  std::vector<geometry_msgs::msg::Pose> all_poses_;  // To accumulate all poses

  struct Transform {
    double x, y, z, roll, pitch, yaw;
  } lidar_to_gnss_transform_, lidar_to_base_link_transform_;

  void publishGpsData();
  sensor_msgs::msg::NavSatFix createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                               double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp);
  geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(double easting, double northing, double altitude,
                                                                double roll, double pitch, double yaw,
                                                                double east_sd, double north_sd, double height_sd,
                                                                double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time timestamp);
  geometry_msgs::msg::PoseStamped createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance);
  Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);
  geometry_msgs::msg::Pose poseWithCovarianceToPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance);
  sensor_msgs::msg::Imu createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                       double z_angular_rate, double x_acceleration, double y_acceleration,
                                       double z_acceleration, double roll, double pitch, double yaw,
                                       double roll_sd, double pitch_sd, double heading_sd);
  void publishTwistMessage(double east_velocity, double north_velocity, double up_velocity,
                           double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time);
};

#endif  // ROS_POSPAC_BRIDGE_ROS_POSPAC_BRIDGE_HPP_
