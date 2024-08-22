#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>  // For M_PI and conversion functions

class GpsPublisher : public rclcpp::Node
{
public:
    GpsPublisher() : Node("ros_pospac_bridge"), last_ros_time_(this->get_clock()->now())
    {
        // Declare parameters
        this->declare_parameter<std::string>("gps_data_file");
        this->declare_parameter<double>("origin_latitude", 0.0);
        this->declare_parameter<double>("origin_longitude", 0.0);
        this->declare_parameter<double>("origin_altitude", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.x", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.y", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.z", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.roll", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.pitch", 0.0);
        this->declare_parameter<double>("lidar_to_gnss_transform.yaw", 0.0);
        this->declare_parameter<double>("lidar_to_base_link_transform.x", 0.0);
        this->declare_parameter<double>("lidar_to_base_link_transform.y", 0.0);
        this->declare_parameter<double>("lidar_to_base_link_transform.z", 0.0);

        this->declare_parameter<double>("lidar_to_base_link_transform.roll", 0.0);
        this->declare_parameter<double>("lidar_to_base_link_transform.pitch", 0.0);
        this->declare_parameter<double>("lidar_to_base_link_transform.yaw", 0.0);

        // Retrieve parameters
        this->get_parameter("gps_data_file", file_path_);
        this->get_parameter("origin_latitude", origin_latitude_);
        this->get_parameter("origin_longitude", origin_longitude_);
        this->get_parameter("origin_altitude", origin_altitude_);

        this->get_parameter("lidar_to_gnss_transform.x", lidar_to_gnss_transform_.x);
        this->get_parameter("lidar_to_gnss_transform.y", lidar_to_gnss_transform_.y);
        this->get_parameter("lidar_to_gnss_transform.z", lidar_to_gnss_transform_.z);
        this->get_parameter("lidar_to_gnss_transform.roll", lidar_to_gnss_transform_.roll);
        this->get_parameter("lidar_to_gnss_transform.pitch", lidar_to_gnss_transform_.pitch);
        this->get_parameter("lidar_to_gnss_transform.yaw", lidar_to_gnss_transform_.yaw);

        this->get_parameter("lidar_to_base_link_transform.x", lidar_to_base_link_transform_.x);
        this->get_parameter("lidar_to_base_link_transform.y", lidar_to_base_link_transform_.y);
        this->get_parameter("lidar_to_base_link_transform.z", lidar_to_base_link_transform_.z);
        this->get_parameter("lidar_to_base_link_transform.roll", lidar_to_base_link_transform_.roll);
        this->get_parameter("lidar_to_base_link_transform.pitch", lidar_to_base_link_transform_.pitch);
        this->get_parameter("lidar_to_base_link_transform.yaw", lidar_to_base_link_transform_.yaw);

        // Convert origin latitude and longitude to UTM
        GeographicLib::UTMUPS::Forward(origin_latitude_, origin_longitude_, origin_zone_, origin_northp_, origin_easting_, origin_northing_);
        initial_altitude_ = origin_altitude_;

        // Initialize publishers
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_fix", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_with_covariance", 10);
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);

        // Start publishing data
        publishGpsData();
    }

private:
    double origin_latitude_, origin_longitude_, origin_altitude_;
    double origin_easting_, origin_northing_;
    double initial_altitude_;
    int origin_zone_;
    bool origin_northp_;
    std::string file_path_;
    rclcpp::Time last_ros_time_;  // To store the ROS time of the last message

    std::vector<geometry_msgs::msg::Pose> all_poses_;  // To accumulate all poses

    struct Transform {
        double x, y, z, roll, pitch, yaw;
    } lidar_to_gnss_transform_, lidar_to_base_link_transform_;

    void publishGpsData()
    {
        std::ifstream file(file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
            return;
        }

        std::string line;
        double last_time = 0.0;
        bool is_first_line = true;

        while (std::getline(file, line) && rclcpp::ok())
        {
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
                    >> east_sd >> north_sd >> height_sd >> roll_sd >> pitch_sd >> heading_sd)
            {
                rclcpp::Time current_ros_time = this->get_clock()->now();
                
                if (is_first_line) {
                    last_ros_time_ = current_ros_time;
                    last_time = time;
                } else {
                    double time_difference = time - last_time;
                    rclcpp::Duration duration = rclcpp::Duration::from_seconds(time_difference);
                    rclcpp::Time target_time = last_ros_time_ + duration;

                    // Spin while waiting for the target time
                    while (rclcpp::ok() && this->get_clock()->now() < target_time) {
                        rclcpp::sleep_for(std::chrono::milliseconds(10));  // Sleep in small increments
                    }

                    last_ros_time_ = target_time;
                    last_time = time;
                }

                // Convert latitude and longitude to UTM
                int zone;
                bool northp;
                double utm_easting, utm_northing;
                GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);

                double local_easting = utm_easting - origin_easting_;
                double local_northing = utm_northing - origin_northing_;
                double relative_altitude = ortho_height - initial_altitude_;

                // Create PoseWithCovarianceStamped from GPS data
                auto pose_msg = createPoseMessage(local_easting, local_northing, relative_altitude, roll, pitch, heading,
                                                  east_sd, north_sd, height_sd, roll_sd, pitch_sd, heading_sd, last_ros_time_);
                
                // Transform pose to base_link frame and update covariance accordingly
                auto transformed_pose_msg = transformPoseToBaseLink(pose_msg);
                
                // Publish the final pose in the base_link frame
                pose_pub_->publish(transformed_pose_msg);

                // Add the pose to the list of all poses
                all_poses_.push_back(poseWithCovarianceToPose(transformed_pose_msg));

                // Create and publish a PoseArray message
                geometry_msgs::msg::PoseArray pose_array_msg;
                pose_array_msg.header.stamp = transformed_pose_msg.header.stamp;
                pose_array_msg.header.frame_id = "base_link";
                pose_array_msg.poses = all_poses_;

                pose_array_pub_->publish(pose_array_msg);

                // Publish IMU data
                auto imu_msg = createImuMessage(last_ros_time_, x_angular_rate, y_angular_rate, z_angular_rate,
                                                x_acceleration, y_acceleration, z_acceleration, roll, pitch, heading,
                                                roll_sd, pitch_sd, heading_sd);
                imu_pub_->publish(imu_msg);

                is_first_line = false;
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }
        file.close();
    }

    geometry_msgs::msg::PoseWithCovarianceStamped transformPoseToBaseLink(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped transformed_pose_msg = pose_msg;
        transformed_pose_msg.header.frame_id = "base_link";

        // Step 1: Apply the GNSS to LiDAR transformation (inverse of the original LiDAR to GNSS transformation)
        tf2::Quaternion lidar_to_gnss_q = toQuaternion(lidar_to_gnss_transform_.roll, lidar_to_gnss_transform_.pitch, lidar_to_gnss_transform_.yaw);
        tf2::Quaternion gnss_to_lidar_q = lidar_to_gnss_q.inverse();
        tf2::Vector3 gnss_to_lidar_translation = -1 * tf2::Vector3(lidar_to_gnss_transform_.x, lidar_to_gnss_transform_.y, lidar_to_gnss_transform_.z);
        
        tf2::Quaternion current_orientation(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);
        tf2::Vector3 current_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z);

        // Apply the inverse quaternion rotation and translation
        tf2::Quaternion transformed_orientation = gnss_to_lidar_q * current_orientation;
        transformed_orientation.normalize();

        tf2::Vector3 transformed_position = current_position + gnss_to_lidar_translation;

        // Step 2: Apply the LiDAR to base_link transformation (unchanged from original)
        tf2::Quaternion lidar_to_base_q = toQuaternion(lidar_to_base_link_transform_.roll, lidar_to_base_link_transform_.pitch, lidar_to_base_link_transform_.yaw);
        tf2::Vector3 lidar_to_base_translation(lidar_to_base_link_transform_.x, lidar_to_base_link_transform_.y, lidar_to_base_link_transform_.z);

        transformed_orientation = lidar_to_base_q * transformed_orientation;
        transformed_orientation.normalize();

        transformed_position = transformed_position + lidar_to_base_translation;

        // Set the transformed orientation and position
        transformed_pose_msg.pose.pose.orientation.x = transformed_orientation.x();
        transformed_pose_msg.pose.pose.orientation.y = transformed_orientation.y();
        transformed_pose_msg.pose.pose.orientation.z = transformed_orientation.z();
        transformed_pose_msg.pose.pose.orientation.w = transformed_orientation.w();

        transformed_pose_msg.pose.pose.position.x = transformed_position.x();
        transformed_pose_msg.pose.pose.position.y = transformed_position.y();
        transformed_pose_msg.pose.pose.position.z = transformed_position.z();

        // Transform the covariance for position and orientation
        tf2::Matrix3x3 rot_matrix(lidar_to_base_q);
        tf2::Matrix3x3 original_covariance_orientation;
        tf2::Matrix3x3 original_covariance_position;

        original_covariance_position.setValue(
            transformed_pose_msg.pose.covariance[0], transformed_pose_msg.pose.covariance[1], transformed_pose_msg.pose.covariance[2],
            transformed_pose_msg.pose.covariance[6], transformed_pose_msg.pose.covariance[7], transformed_pose_msg.pose.covariance[8],
            transformed_pose_msg.pose.covariance[12], transformed_pose_msg.pose.covariance[13], transformed_pose_msg.pose.covariance[14]);

        original_covariance_orientation.setValue(
            transformed_pose_msg.pose.covariance[21], transformed_pose_msg.pose.covariance[22], transformed_pose_msg.pose.covariance[23],
            transformed_pose_msg.pose.covariance[27], transformed_pose_msg.pose.covariance[28], transformed_pose_msg.pose.covariance[29],
            transformed_pose_msg.pose.covariance[33], transformed_pose_msg.pose.covariance[34], transformed_pose_msg.pose.covariance[35]);

        tf2::Matrix3x3 transformed_covariance_orientation = rot_matrix * original_covariance_orientation * rot_matrix.transpose();
        tf2::Matrix3x3 transformed_covariance_position = rot_matrix * original_covariance_position * rot_matrix.transpose();

        // Update covariance matrix with transformed values
        transformed_pose_msg.pose.covariance[0] = transformed_covariance_position[0][0];
        transformed_pose_msg.pose.covariance[1] = transformed_covariance_position[0][1];
        transformed_pose_msg.pose.covariance[2] = transformed_covariance_position[0][2];
        transformed_pose_msg.pose.covariance[6] = transformed_covariance_position[1][0];
        transformed_pose_msg.pose.covariance[7] = transformed_covariance_position[1][1];
        transformed_pose_msg.pose.covariance[8] = transformed_covariance_position[1][2];
        transformed_pose_msg.pose.covariance[12] = transformed_covariance_position[2][0];
        transformed_pose_msg.pose.covariance[13] = transformed_covariance_position[2][1];
        transformed_pose_msg.pose.covariance[14] = transformed_covariance_position[2][2];

        transformed_pose_msg.pose.covariance[21] = transformed_covariance_orientation[0][0];
        transformed_pose_msg.pose.covariance[22] = transformed_covariance_orientation[0][1];
        transformed_pose_msg.pose.covariance[23] = transformed_covariance_orientation[0][2];
        transformed_pose_msg.pose.covariance[27] = transformed_covariance_orientation[1][0];
        transformed_pose_msg.pose.covariance[28] = transformed_covariance_orientation[1][1];
        transformed_pose_msg.pose.covariance[29] = transformed_covariance_orientation[1][2];
        transformed_pose_msg.pose.covariance[33] = transformed_covariance_orientation[2][0];
        transformed_pose_msg.pose.covariance[34] = transformed_covariance_orientation[2][1];
        transformed_pose_msg.pose.covariance[35] = transformed_covariance_orientation[2][2];

        return transformed_pose_msg;
    }

    tf2::Quaternion toQuaternion(double roll, double pitch, double yaw, bool angles_in_degrees = false)
    {
        if (angles_in_degrees) {
            roll = roll * M_PI / 180.0;
            pitch = pitch * M_PI / 180.0;
            yaw = yaw * M_PI / 180.0;
        }

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        return q;
    }

    sensor_msgs::msg::NavSatFix createGpsMessage(double latitude, double longitude, double ellipsoid_height,
                                                 double east_sd, double north_sd, double height_sd, rclcpp::Time timestamp)
    {
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = timestamp;
        gps_msg.header.frame_id = "base_link"; // Changed to base_link
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

    geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(double easting, double northing, double altitude,
                                                                    double roll, double pitch, double yaw,
                                                                    double east_sd, double north_sd, double height_sd,
                                                                    double roll_sd, double pitch_sd, double yaw_sd, rclcpp::Time timestamp)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = "base_link"; // Changed to base_link

        // Set position
        pose_msg.pose.pose.position.x = easting;
        pose_msg.pose.pose.position.y = northing;
        pose_msg.pose.pose.position.z = altitude;

        // Convert from Euler angles (assuming they are in degrees) to quaternion
        tf2::Quaternion q = toQuaternion(roll, pitch, yaw, true);
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

    geometry_msgs::msg::Pose poseWithCovarianceToPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_covariance)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = pose_with_covariance.pose.pose.position;
        pose.orientation = pose_with_covariance.pose.pose.orientation;
        return pose;
    }

    sensor_msgs::msg::Imu createImuMessage(rclcpp::Time timestamp, double x_angular_rate, double y_angular_rate,
                                           double z_angular_rate, double x_acceleration, double y_acceleration,
                                           double z_acceleration, double roll, double pitch, double yaw,
                                           double roll_sd, double pitch_sd, double heading_sd,
                                           double angular_velocity_sd = 1.0, double linear_acceleration_sd = 1.0)
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = timestamp;
        imu_msg.header.frame_id = "base_link"; // Changed to base_link

        tf2::Quaternion q = toQuaternion(roll, pitch, yaw, true);  // Assuming angles are in degrees
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

        imu_msg.orientation_covariance[0] = roll_sd > 0 ? roll_sd * roll_sd : 1.0;
        imu_msg.orientation_covariance[4] = pitch_sd > 0 ? pitch_sd * pitch_sd : 1.0;
        imu_msg.orientation_covariance[8] = heading_sd > 0 ? heading_sd * heading_sd : 1.0;

        imu_msg.angular_velocity_covariance[0] = angular_velocity_sd * angular_velocity_sd;
        imu_msg.angular_velocity_covariance[4] = angular_velocity_sd * angular_velocity_sd;
        imu_msg.angular_velocity_covariance[8] = angular_velocity_sd * angular_velocity_sd;

        imu_msg.linear_acceleration_covariance[0] = linear_acceleration_sd * linear_acceleration_sd;
        imu_msg.linear_acceleration_covariance[4] = linear_acceleration_sd * linear_acceleration_sd;
        imu_msg.linear_acceleration_covariance[8] = linear_acceleration_sd * linear_acceleration_sd;

        return imu_msg;
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisher>());
    rclcpp::shutdown();
    return 0;
}
