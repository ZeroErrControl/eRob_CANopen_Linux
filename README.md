# eRob Master - CANopen ROS 2 Controller

eRob Master is a ROS 2-based CANopen motor controller used to control motors that support the CANopen protocol via CAN bus. This controller supports position and velocity control, and provides a series of ROS 2 interfaces for motor monitoring and control.

## Features

- CANopen protocol communication support
- Position and velocity control support
- ROS 2 topics and service interfaces
- Real-time monitoring of motor status, position, and velocity
- Support for motor initialization, start, stop, and reset
- Profile parameter settings support (velocity, acceleration, deceleration)

## Installation

### Prerequisites

- ROS 2 (Humble or higher version recommended)
- CAN interface (e.g., Socket-CAN adapter)
- Motors supporting CANopen protocol

### Build and Install

1. Create workspace

   ```bash
   mkdir -p ~/erob_ws/src
   cd ~/erob_ws/src
   ```

2. Clone repository

   ```bash
   git clone git@github.com:ZeroErrControl/eRob_CANopen_Linux.git
   ```

3. Build

   ```bash
   cd ~/erob_ws
   colcon build 
   ```

4. Launch node

   ```bash
   source install/setup.bash
   ros2 launch erob_master erob_master.launch.py
   ```

## Configure CAN Interface

### 1. Load CAN modules

```bash
sudo modprobe can
sudo modprobe can_raw
```

### 2. Configure CAN interface (example for can0 with 1Mbps baudrate)

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 3. Check CAN interface status

```bash
ip -details link show can0
```

## Usage

### 1. Launch Node

Launch with default parameters

```bash
ros2 launch erob_master canopen_ros2.launch.py
```

### 2. Launch with custom parameters
```bash
ros2 launch erob_master canopen_ros2.launch.py can_interface:=can0 node_id:=2 auto_start:=true
```

## Controlling the Motor

### 1. Position Control

Control motor position by publishing to /target_position topic:

- Move to 90 degrees

```bash
ros2 topic pub /target_position std_msgs/msg/Float32 "data: 90.0" --once
```

- Move to 180 degrees

```bash
ros2 topic pub /target_position std_msgs/msg/Float32 "data: 180.0" --once
```

### 2. Velocity Control

Control motor velocity by publishing to /target_velocity topic:

- Set velocity to 10 degrees/second

```bash
ros2 topic pub /target_velocity std_msgs/msg/Float32 "data: 10.0" --once
```

- Set velocity to -10 degrees/second

```bash
ros2 topic pub /target_velocity std_msgs/msg/Float32 "data: -10.0" --once
```

## Service Interfaces

### 1. Start Motor

```bash
ros2 service call /start_erob std_srvs/srv/Trigger
```

### 2. Stop Motor

```bash
ros2 service call /stop_erob std_srvs/srv/Trigger
```

### 3. Reset Motor

```bash
ros2 service call /reset_erob std_srvs/srv/Trigger
```

## Setting Motor Mode

- Set to position mode

```bash
ros2 service call /set_erob_mode std_srvs/srv/SetBool "data: true"
```

- Set to velocity mode

```bash
ros2 service call /set_erob_mode std_srvs/srv/SetBool "data: false"
```

## Monitor Motor Status

```bash
ros2 topic echo /erob_status
```

## View Motor Position

```bash
ros2 topic echo /erob_position
```

## View Motor Velocity

```bash
ros2 topic echo /erob_velocity
```

## Topic List

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| /target_position | std_msgs/msg/Float32 | Set target position (degrees) |
| /target_velocity | std_msgs/msg/Float32 | Set target velocity (degrees/second) |
| /erob_status | std_msgs/msg/String | Motor status information |
| /erob_position | std_msgs/msg/Float32 | Current position (degrees) |
| /erob_velocity | std_msgs/msg/Float32 | Current velocity (degrees/second) |

## Service List

| Service Name | Service Type | Description |
| ------------ | ------------ | ----------- |
| /start_erob | std_srvs/srv/Trigger | Start motor |
| /stop_erob | std_srvs/srv/Trigger | Stop motor |
| /reset_erob | std_srvs/srv/Trigger | Reset motor |
| /set_erob_mode | std_srvs/srv/SetBool | Set motor mode (true: position mode, false: velocity mode) |

## Parameter List

| Parameter Name | Type | Default Value | Description |
| -------------- | ---- | ------------- | ----------- |
| can_interface | string | "can0" | CAN interface name |
| node_id | int | 2 | CANopen node ID |
| auto_start | bool | true | Whether to auto-start motor |
| profile_velocity | int | 5 | Profile velocity (degrees/second) |
| profile_acceleration | int | 5 | Profile acceleration (degrees/second²) |
| profile_deceleration | int | 5 | Profile deceleration (degrees/second²) |

## Troubleshooting

### Cannot Connect to CAN Interface

- Check if CAN interface is properly configured: ip -details link show can0
- Ensure CAN interface is up: sudo ip link set up can0
- Verify CAN bus connections are correct

### Motor Not Responding to Commands

- Check if motor power is connected
- Confirm node ID is correct
- Monitor CAN bus communication using candump: candump can0
- Check motor status: ros2 topic echo /erob_status

### Cannot Switch Operation Mode

- Some motors may not support standard CiA402 operation mode switching. In this case, the controller will attempt to work in the current mode.

### Advanced Usage

- Using candump to monitor CAN communication
Install can-utils

```bash
sudo apt-get install can-utils
```

### Monitor CAN bus

```bash
candump can0
```

## License

Apache License 2.0
