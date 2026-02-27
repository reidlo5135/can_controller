# ros-vehicle-core

ROS 2 vehicle control core workspace.

This repository contains core packages for CAN communication, motion control, and motion validation used in vehicle platforms.

## Overview

High-level flow:

- `motion_controller` converts control inputs into vehicle control commands.
- `can_manager` bridges ROS topics and CAN frames in both directions.
- `motion_validation` validates and corrects vehicle state based on runtime data.

## Repository Structure

- `can_manager`: ROS <-> CAN bridge node
  - Details: [can_manager/README.md](can_manager/README.md)
- `motion_controller`: vehicle motion control node
  - Details: [motion_controller/README.md](motion_controller/README.md)
- `motion_validation`: vehicle state validation and correction node
- `can_utils`: shared CAN utility library
- `can_dbc`: DBC-based CAN signal definitions

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `colcon` and `ament_cmake`
- SocketCAN-capable environment for target hardware/simulation

The following external packages may also be required in your workspace:

- `map_handle_msgs`
- `robot_status_msgs`

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Configuration

Default parameter files:

- `can_manager/config/can_manager.yaml`
- `motion_controller/config/motion_controller.yaml`
- `motion_validation/config/motion_validation.yaml`

Tune topic names, CAN channels, and vehicle parameters for your platform.
