# teleop_ack_rc

An opinionated ROS2 C++ node for converting RC serial packets into Ackermann drive commands, optimized for Project Phoenix and ISC standards.

## Instructions

1. Clone repo inside your workspace src directory (Ex. phnx_ws/src)
2. `rosdep install --from-paths . --ignore-src -r -y` to install deps
3. `colcon build --packages-select teleop_ack_rc` to build the package
4. `colcon test --packages-select teleop_ack_rc --event-handlers console_direct+` to run unit tests

## Dependencies
- rclcpp
- sensor_msgs
- ackermann_msgs
- libserial (External library for Arduino communication)

## Features
- Unit tests for velocity mapping and jitter suppression
- Multithreaded executor for high-frequency serial I/O
- Hardware-specific calibration for Project Phoenix powertrain

## Hardware Calibration & Benchmarks
The node is configured for a target top speed of 15.0 mph (6.7056 m/s).

| Input Device | Observed Max Throttle | Actual Top Speed | Notes |
| :--- | :--- | :--- | :--- |
| 8BitDo (Wukong) | 1.0 (100%) | 6.70 m/s | Digital precision |
| RC Transmitter | 0.94 (94%) | 6.33 m/s | Range limited by 7yo pots |

### Serial Configuration
- **Baud Rate:** 115200
- **Polling Rate:** 50Hz (20ms)
- **Neutral Pulse:** 1500 (Subtrim calibrated)

## File structure

.
├── include
│   └── teleop_ack_rc
│       └── TeleopAckRc_node.hpp
├── package.xml
├── README.md
├── src
│   ├── teleop_ack_rc.cpp        # Main function and executor setup
│   └── teleop_ack_rc_node.cpp   # Node logic and serial processing
└── tests
└── unit.cpp                 # Logic verification and edge cases


### Development Notes
- **Jitter Management:** If significant jitter is observed at neutral, check Arduino interrupt timing. The `unit.cpp` tests include a deadzone validation (default 0.05) to suppress minor signal noise.
- **Device Persistence:** When using the 8BitDo controller, utilize the `/dev/input/by-id/` path in the joy_node launch to prevent keyboard interference.
Would you like me to generate the src/teleop_ack_rc.cpp file now so you can finish the split between the main function and the node logic?