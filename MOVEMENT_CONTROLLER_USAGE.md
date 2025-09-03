# Spacecraft Movement Controller Usage Guide

## Overview
The spacecraft movement controller provides a comprehensive ROS-based interface for controlling spacecraft movement with precise stopping capabilities.

## Files Created
- `spacecraft_movement_controller.py` - Main ROS controller node
- `movement_client.py` - Easy-to-use client for sending commands
- `corrected_movement_control.py` - Standalone corrected movement script

## Quick Start

### 1. Start the Controller
```bash
# Terminal 1 - Start the main controller
python3 spacecraft_movement_controller.py
```

### 2. Send Commands (Choose one method)

#### Method A: Using the Client Script (Easiest)
```bash
# Terminal 2 - Send commands using the client

# Controlled movement (recommended)
python3 movement_client.py controlled 0.1 1.5

# Gentle pulsed movement
python3 movement_client.py gentle 0.05 3

# Precise short movement
python3 movement_client.py precise 0.08 0.5 0.4

# Emergency stop
python3 movement_client.py emergency

# Monitor status
python3 movement_client.py monitor
```

#### Method B: Direct ROS Commands
```bash
# Controlled movement
ros2 topic pub /spacecraft/movement/command std_msgs/String '{"action": "controlled_movement", "params": {"power": 0.1, "time": 1.5}}'

# Emergency stop
ros2 topic pub /spacecraft/emergency_stop std_msgs/Bool 'data: true'

# Monitor status
ros2 topic echo /spacecraft/movement/status
```

## Features

### 🎯 **Movement Types**
1. **Controlled Movement**: Optimized forward-brake sequence with cleanup pulses
2. **Gentle Movement**: Multiple small pulses with gradual braking
3. **Precise Movement**: Very short, precise movements for fine positioning

### 🛑 **Safety Features**
- **Emergency Stop**: Instantly stops all movement
- **Movement Lock**: Prevents overlapping movements
- **Status Monitoring**: Real-time movement status
- **Error Handling**: Graceful error recovery

### 📡 **ROS Topics**

#### Publishers (Controller → You)
- `/spacecraft/movement/status` - Current movement status
- `/spacecraft/movement/completed` - Movement completion notifications

#### Subscribers (You → Controller)
- `/spacecraft/movement/command` - JSON movement commands
- `/spacecraft/cmd_vel` - Manual Twist control (like gamepad)
- `/spacecraft/emergency_stop` - Emergency stop commands

### ⚙️ **Configuration**
```bash
# Configure default parameters
python3 movement_client.py configure default_power=0.15 default_time=2.0 cleanup_power=0.02
```

## Movement Parameters

### Controlled Movement
- `power`: Thrust power (0.0-1.0, default: 0.1)
- `time`: Movement duration in seconds (default: 1.5)

### Gentle Movement  
- `pulse_power`: Power per pulse (default: 0.05)
- `num_pulses`: Number of forward pulses (default: 3)
- `pulse_time`: Duration per pulse (default: 0.3)

### Precise Movement
- `power`: Thrust power (default: 0.08)
- `forward_time`: Forward thrust time (default: 0.5)
- `brake_time`: Brake thrust time (default: 0.4)

## Example Workflow

```bash
# Terminal 1: Start controller
python3 spacecraft_movement_controller.py

# Terminal 2: Monitor status
python3 movement_client.py monitor

# Terminal 3: Send commands
python3 movement_client.py controlled 0.12 2.0   # Move with 12% power for 2 seconds
python3 movement_client.py stop                  # Stop if needed
python3 movement_client.py gentle 0.03 5         # Very gentle movement
```

## Troubleshooting

### Movement Not Stopping Completely
- The controller includes automatic cleanup pulses
- If still drifting, reduce brake_time or increase cleanup pulses
- Use emergency stop for immediate halt

### Permission Issues  
- Files owned by root? Create new copies with correct ownership
- Run inside Docker container if needed [[memory:5898928]]

### ROS Connection Issues
- Ensure ROS2 is sourced: `source /opt/ros/humble/setup.bash`
- Check topic connections: `ros2 topic list | grep spacecraft`

## Tips
1. **Start gentle**: Use low power (0.05-0.1) for initial testing
2. **Monitor status**: Always run the monitor in a separate terminal
3. **Emergency ready**: Keep emergency stop command ready
4. **Gradual increase**: Slowly increase power/time as needed
5. **Use controlled**: The "controlled" movement type works best for most cases

## Advanced Usage

### Manual Control with Gamepad/Keyboard
```bash
# Install teleop tools
sudo apt install ros-humble-teleop-twist-keyboard

# Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/spacecraft/cmd_vel
```

### Integration with Other Nodes
The controller publishes standard ROS messages, so you can integrate it with:
- Path planning nodes
- Navigation systems  
- Mission control interfaces
- Autonomous control loops

---
**Happy flying! 🚀**
