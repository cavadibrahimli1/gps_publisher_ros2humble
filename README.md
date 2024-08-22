# GPS Publisher

## Overview
The `GpsPublisher` is a ROS 2 node that reads GPS and IMU data from a file and publishes the information as ROS messages. It simulates a real-time GPS and IMU data stream, publishing the data with corresponding timestamps as if it were being received live.

This node is especially useful for testing and development purposes, where real GPS/IMU hardware is not available.

## Features
- **GPS Data Publishing**: Publishes `sensor_msgs/NavSatFix` messages representing GPS data.
- **IMU Data Publishing**: Publishes `sensor_msgs/Imu` messages representing IMU data.
- **Pose Data Publishing**: Publishes `geometry_msgs/PoseWithCovarianceStamped` messages for the vehicle's pose.
- **Conversion of Latitude/Longitude to UTM**: Converts GPS coordinates from latitude/longitude to UTM coordinates.
- **Quaternion Calculation**: Converts roll, pitch, and yaw angles into quaternions for use in ROS messages.

## Prerequisites
Ensure that you have the following dependencies installed:
- ROS 2
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `tf2`
- `tf2_geometry_msgs`
- `GeographicLib`

These dependencies are also automatically handled via `ament_cmake`.

## Installation
1. Clone the repository into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone <repository_url> gps_publisher
    ```

2. Navigate to your workspace and build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select gps_publisher
    ```

3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Configuration
The node parameters are configured using a YAML file. A sample configuration file is provided in the `config` directory. The parameters include:

- `gps_data_file`: Path to the file containing the GPS and IMU data.
- `origin_latitude`: Latitude of the origin point.
- `origin_longitude`: Longitude of the origin point.
- `origin_altitude`: Altitude of the origin point.

### Sample YAML Configuration (`config/config.yaml`)
```yaml
gps_publisher:
  ros__parameters:
    gps_data_file: "/path/to/your/data/file.txt"
    origin_latitude: 41.026797780921
    origin_longitude: 28.980012541159
    origin_altitude: 42.743
```

## Usage
To launch the node, use the provided launch file. The launch file will use the parameters specified in the YAML configuration file.

```bash
ros2 launch gps_publisher gps_publisher.launch.py param_file:=/path/to/config.yaml
```
