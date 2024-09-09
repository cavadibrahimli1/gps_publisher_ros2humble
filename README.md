# ROS PosPac Bridge

## Overview



The `ros_pospac_bridge` is a ROS 2 package designed to process GPS data and convert it into various ROS message formats, including PoseWithCovariance, IMU, and Twist. The package also transforms poses into a consistent map frame using tf2, making it useful for localization and navigation tasks in robotic systems.

## Prerequisites

- ROS 2 Galactic (or newer)
- GeographicLib
- Eigen

Make sure that you have ROS 2 installed and sourced. Install any necessary dependencies before proceeding.

## Installation

### 1. Clone the Repository

```bash
$ git clone <repository_url>
```

### 2. Build the Package

```bash
$ cd ~/ros2_ws
$ colcon build --packages-select ros_pospac_bridge
```

### 3. Source the Workspace

```bash
$ source install/setup.bash
```

## Configuration

The ros_pospac_bridge node reads its configuration from a YAML file located in the config directory. You can update the file path for GPS data and other parameters in the ros_pospac_bridge.config.yaml file.


ros_pospac_bridge:
  ros__parameters:
    gps_data_file: "/home/javadibrahimli/ros2_ws/route2.txt"
    mgrs_origin: "35TPF6645943620"
    origin_altitude: 42.743

    # Publisher Control (True/False Flags)
    start_gps_publisher: true
    start_imu_publisher: true
    start_pose_publisher: true
    start_pose_array_publisher: true
    start_twist_publisher: true
    start_pose_stamped_publisher: true

    # Transformation Settings
    lidar_to_gnss_transform:
      x: 0.0
      y: 0.0
      z: -0.08
      roll: 0.007237
      pitch: -0.0072323
      yaw: 3.1334945

    lidar_to_base_link_transform:
      x: 0.96
      y: -0.01
      z: 1.1
      roll: 0.0
      pitch: 0.0
      yaw: -3.0965926


- gps_data_file: The path to the GPS data file that will be processed.
- mgrs_origin: The origin in MGRS coordinates used as a reference for transformations.
- origin_altitude: The altitude at the origin point.
- Transformation Settings: These settings define the physical transformations between the LiDAR, GNSS, and base link frames.





## Running the Package
1. Launch the ROS PosPac Bridge Node

```bash
$ ros2 launch ros_pospac_bridge ros_pospac_bridge.launch.xml
```
Expected Terminal Output:

```bash
[INFO] [ros_pospac_bridge]: GPS data file path: /home/javadibrahimli/ros2_ws/route2.txt
[INFO] [ros_pospac_bridge]: MGRS Origin: 35TPF6645943620 converted to UTM: easting 66457.90214, northing 43622.34122
[INFO] [ros_pospac_bridge]: Attempting to open GPS data file at: /home/javadibrahimli/ros2_ws/route2.txt
[INFO] [ros_pospac_bridge]: Processing line: <line_data_here>
[INFO] [ros_pospac_bridge]: Parsed line successfully.
[INFO] [ros_pospac_bridge]: MGRS location: <MGRS_location_here>
[INFO] [ros_pospac_bridge]: Published GPS, Pose, IMU, and Twist messages.
```

2. Launch Autoware Simulation

```bash
$ source ~/autoware/install/setup.bash
$ ros2 launch autoware_launch logging_simulator.launch.xml map_path:=/home/javadibrahimli/autoware_map/test_route2 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
Expected Terminal Output:

```bash
[INFO] [Autoware]: Loading map from /home/javadibrahimli/autoware_map/test_route2
[INFO] [Autoware]: Vehicle model set to sample_vehicle
[INFO] [Autoware]: Sensor model set to sample_sensor_kit
[INFO] [Autoware]: Simulator initialized successfully
```

## Notes
Replace <repository_url> with the actual URL of your repository.
Replace <line_data_here> and <MGRS_location_here> with the actual data being processed by the ros_pospac_bridge node.

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)



