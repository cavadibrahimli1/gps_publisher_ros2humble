#ifndef ROS_POSPAC_BRIDGE_HPP
#define ROS_POSPAC_BRIDGE_HPP

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
#include <memory>

class RosPospacBridge : public rclcpp::Node {
public:
  RosPospacBridge();

private:
  void CreatePublishGpsData();
  void publishMapToBaseLinkTransform(const geometry_msgs::msg::Pose& gnss_ins_pose, const rclcpp::Time& timestamp);
  Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);
  sensor_msgs::msg::NavSatFix createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                               double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp);
  geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(double latitude, double longitude, double altitude,
                                                                  double roll, double pitch, double yaw,
                                                                  double east_sd, double north_sd, double height_sd,
                                                                  double roll_sd, double pitch_sd, double yaw_sd, 
                                                                  rclcpp::Time timestamp);
  geometry_msgs::msg::PoseStamped createPoseStampedMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance);
  sensor_msgs::msg::Imu createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                         double z_angular_rate, double x_acceleration, double y_acceleration,
                                         double z_acceleration, double roll, double pitch, double yaw,
                                         double roll_sd, double pitch_sd, double heading_sd);
  void publishTwistMessage(double east_velocity, double north_velocity, double up_velocity,
                           double x_angular_rate, double y_angular_rate, double z_angular_rate, rclcpp::Time sensor_time);

  // New method declarations
  void initializeTransformListener();
  void getCalibrations();
  geometry_msgs::msg::Pose transformPoseToBaseLink(const geometry_msgs::msg::Pose& pose);

  // Publisher pointers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;

  // bool enable_gps_pub_;
  // bool enable_imu_pub_;
  // bool enable_pose_pub_;
  // bool enable_pose_array_pub_;
  // bool enable_twist_pub_;
  // bool enable_pose_stamped_pub_;
  // bool enable_tf_pub_;

  double origin_easting_;
  double origin_northing_;

  std::string file_path_;

  std::vector<geometry_msgs::msg::Pose> all_poses_;  // To accumulate all poses
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::Transform lidar_to_gnss_transform_;  // Updated to reflect lidar-to-GNSS
  tf2::Transform base_link_to_lidar_transform_;  // New member variable
};

#endif  // ROS_POSPAC_BRIDGE_HPP
