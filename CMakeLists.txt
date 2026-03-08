cmake_minimum_required(VERSION 3.8)
project(teleop_ack_rc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find LibSerial (External Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(include)

# 1. Create the Shared Library (The Node Logic)
add_library(teleop_ack_rc_node SHARED
  src/teleop_ack_rc_node.cpp
)

target_include_directories(teleop_ack_rc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS dependencies to the library
ament_target_dependencies(teleop_ack_rc_node
  rclcpp
  rclcpp_components
  ackermann_msgs
  sensor_msgs
  std_msgs
)

# Link the external serial library
target_link_libraries(teleop_ack_rc_node ${LIBSERIAL_LIBRARIES})

# 2. Create the Executable (The Main Entry Point)
add_executable(teleop_ack_rc src/teleop_ack_rc.cpp)
target_link_libraries(teleop_ack_rc teleop_ack_rc_node)

# 3. Install Rules so 'ros2 run' can find it
install(TARGETS
  teleop_ack_rc_node
  teleop_ack_rc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# --- Testing Block ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Register the test
  ament_add_gtest(rc_logic_test test/unit.cpp)
  
  # Link to your node's logic so the test can see your functions
  target_link_libraries(rc_logic_test teleop_ack_rc_node)
  
  # Ensure the test finds your headers and ROS dependencies
  target_include_directories(rc_logic_test PUBLIC include)
  ament_target_dependencies(rc_logic_test
    rclcpp
    ackermann_msgs
    sensor_msgs
  )
endif()

ament_package()