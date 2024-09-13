![image](https://github.com/user-attachments/assets/8c54b1b7-dc29-4d02-b05b-a0fb007e1a9b)

# ros_pospac_bridge

`ros_pospac_bridge` is a ROS2 package designed to bridge GPS and IMU data streams into a variety of ROS topics for navigation, localization, and mapping applications. The package reads data from a GPS file, transforms coordinates, and publishes the information in various formats suitable for different ROS-based applications.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
  - [Launch File](#launch-file)
  - [Configuration](#configuration)
  - [Running the Node](#running-the-node)
- [Nodes](#nodes)
  - [ros_pospac_bridge Node](#ros_pospac_bridge-node)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)
- [Coordinate Systems](#coordinate-systems)
- [Configuration in Detail](#configuration-in-detail)
  - [Publishers](#publishers)
  - [Calibration](#calibration)
  - [Advanced Settings](#advanced-settings)
- [Dependencies](#dependencies)
- [Building and Running Tests](#building-and-running-tests)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

The `ros_pospac_bridge` package provides a seamless way to ingest GPS and IMU data and publish it in formats compatible with common ROS messages, such as `sensor_msgs/NavSatFix`, `sensor_msgs/Imu`, and various pose messages for localization tasks. It includes built-in calibration parameters for integrating LIDAR, GNSS, and other sensor data.

Key features:

- Reading GPS data from custom files.
- Publishing GPS, IMU, and pose data in ROS topics.
- Managing transformations between various frames, including base_link, map, and sensor frames.
- Simple integration into any ROS2 environment.

## Installation

To install and build the package, follow the steps below:

1. **Clone the repository**:

   ```bash
   git clone https://github.com/cavadibrahimli1/gps_publisher_ros2humble
   ```

2. **Install dependencies**:
   Ensure that all required dependencies are installed by running the following command:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   Once the repository is cloned, navigate to your ROS workspace and build the package:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the environment**:
   After building, you need to source the environment to use the package:

   ```bash
   source install/setup.bash
   ```

## Usage

### Launch File and Autoware

To simplify the usage, the package comes with a pre-configured launch file that handles node startup and parameter loading. To launch the node:

```bash
ros2 launch ros_pospac_bridge ros_pospac_bridge.launch.xml
```

```bash
source autoware/install/setup.bash 
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=/home/javadibrahimli/autoware_map/test_route2 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

This will start the node using the parameters defined in the `ros_pospac_bridge.config.yaml` file, located in the `config` folder.

### Configuration

The configuration file, `ros_pospac_bridge.config.yaml`, defines all the important parameters for the node, including the input file paths, calibration parameters, and enabled publishers.

To customize the configuration, open the file and adjust the parameters as needed:

```bash
nano config/ros_pospac_bridge.config.yaml
```

For example, if you need to change the GPS data file path, modify the `gps_data_file` entry:

```yaml
gps_data_file: "/path/to/your/gps_data.txt"
```

### Running the Node

After configuring the node and parameters, you can run the node using the command above, or manually as follows:

```bash
ros2 run ros_pospac_bridge ros_pospac_bridge
```

This will execute the node with the settings provided in the configuration file.

## Nodes

### ros_pospac_bridge Node

The core of this package is the `ros_pospac_bridge` node, which handles reading GPS data from the specified file, applying transformations, and publishing the processed data to the relevant ROS topics.

#### Published Topics

- `/ros_pospac_bridge/gps_fix` (sensor_msgs/NavSatFix): Publishes GPS data in the `NavSatFix` format, including latitude, longitude, and altitude.
- `/ros_pospac_bridge/imu_data` (sensor_msgs/Imu): IMU data including orientation, angular velocity, and linear acceleration.
- `/ros_pospac_bridge/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped): Estimated pose with covariance data for better localization accuracy.
- `/ros_pospac_bridge/pose_array` (geometry_msgs/PoseArray): A set of poses used for multi-pose tracking.
- `/ros_pospac_bridge/pose_stamped` (geometry_msgs/PoseStamped): A single pose with timestamp.
- `/ros_pospac_bridge/twist_with_covariance` (geometry_msgs/TwistWithCovarianceStamped): Velocity estimation with covariance.
- `tf` (TF2 Transform): Publishes static and dynamic transformations between frames like `map` to `base_link`.

## Coordinate Systems

The node uses the MGRS coordinate system to define the origin. You can specify the MGRS origin and altitude in the configuration file:

```yaml
mgrs_origin: "35TPF6645943620"  # Replace with your MGRS origin
origin_altitude: 42.743  # Origin altitude in meters
```

This ensures accurate transformation between GPS coordinates and ROS frame data.

## Configuration in Detail

### Publishers

The `ros_pospac_bridge` node allows you to enable or disable specific publishers for various topics. This can be configured in the YAML file under the `publishers` section:

```yaml
publishers:
  nav_sat_fix:
    enable: true
    topic: "/ros_pospac_bridge/gps_fix"
  imu:
    enable: true
    topic: "/ros_pospac_bridge/imu_data"
  pose_with_covariance_stamped:
    enable: true
    topic: "/ros_pospac_bridge/pose_with_covariance"
  pose_array:
    enable: true
    topic: "/ros_pospac_bridge/pose_array"
  tf:
    enable: true
```

By setting `enable: false` for any publisher, you can disable that specific topic.

### Calibration

The `ros_pospac_bridge` supports calibration between different sensors (e.g., LIDAR and GNSS). Calibration parameters are specified in the `calibration` section:

```yaml
calibration:
  lidar_to_gnss:
    x: 0.0
    y: 0.0
    z: -0.08
    roll: 0.007237
    pitch: -0.0072323
    yaw: 3.1334945
  base_link_to_lidar:
    x: 0.96
    y: -0.01
    z: 1.1
    roll: 0.0
    pitch: 0.0
    yaw: -3.0965926
```

Adjust these parameters to reflect your sensor setup for accurate data integration.

![image](https://github.com/user-attachments/assets/3dc45037-cdee-4b03-bd15-9b9f2ef703ae)

### Advanced Settings

For more advanced users, additional parameters like update frequency, filtering options, and dynamic transformations can be added to the configuration file.

## Dependencies

The following ROS2 and third-party packages are required for `ros_pospac_bridge`:

- `rclcpp`: Core ROS2 client library.
- `sensor_msgs`: For IMU and GPS message types.
- `geometry_msgs`: For pose and twist message types.
- `tf2` and `tf2_geometry_msgs`: For transformations between coordinate frames.
- `GeographicLib`: Used for geographic calculations like transformations between GPS and MGRS.

Ensure all dependencies are installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Building and Running Tests

This package includes unit tests for validating GPS and IMU data parsing, transformations, and message publishing. To build and run the tests:

1. Build with testing enabled:

   ```bash
   colcon build --cmake-args -DBUILD_TESTING=ON
   ```

2. Run the tests:

   ```bash
   colcon test
   ```

Test results will be reported in the terminal.

## Troubleshooting

- **Node not publishing topics**: Ensure that the publishers are enabled in the configuration file and that the correct topics are being subscribed to in your ROS environment.
- **Coordinate transformations inaccurate**: Check the calibration parameters and MGRS origin. Improper calibration values can lead to erroneous transformations.
- **Dependency errors**: If any dependencies are missing, use `rosdep install` to automatically resolve and install them.

## Contributing

We welcome contributions from the community! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description of the changes.

Please ensure your code passes the tests before submitting.
