cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring ros_bridge")

# Include core files that are converted + essential files for espeleo_bridge
set(TARGET_SRC
	"src/bridge.cpp"
	"src/entry_publisher.cpp"       # Converted to ROS 2
	"src/entry_subscriber.cpp"      # Converted to ROS 2
	"src/joint_state_publisher.cpp" # Converted to ROS 2
	"src/joint_state_subscriber.cpp" # Converted to ROS 2
	"src/kacanopen_hardware.cpp"    # ROS 2 hardware interface - now fully implemented
	"src/kacanopen_manager.cpp"     # ROS 2 manager - now fully implemented
	"src/kacanopen_motor.cpp"       # ROS 2 motor - now fully implemented
	"src/test_entry_publisher.cpp"  # Test files - converted to ROS 2
	"src/test_joint_state_publisher.cpp" # Test files - converted to ROS 2
	"src/test_joint_state_subscriber.cpp" # Test files - converted to ROS 2
)

# Create library
set(TARGET_NAME kacanopen_ros_bridge)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${TARGET_NAME} kacanopen_master)

# ROS 2 dependencies for library
ament_target_dependencies(${TARGET_NAME}
	rclcpp
	std_msgs
	sensor_msgs
	std_srvs
	controller_manager
	hardware_interface
	transmission_interface
	diagnostic_updater
)

# Install
kacanopen_install(${TARGET_NAME} TRUE)

# ROS 2 hardware interface node - now fully implemented and enabled
add_executable(kacanopen_roscontrol_node node/kacanopen_node.cpp)
set_property(TARGET kacanopen_roscontrol_node PROPERTY CXX_STANDARD 17)
target_link_libraries(kacanopen_roscontrol_node kacanopen_core kacanopen_master kacanopen_ros_bridge)

# ROS 2 dependencies for executable
ament_target_dependencies(kacanopen_roscontrol_node
	rclcpp
	std_msgs
	sensor_msgs
	std_srvs
	controller_manager
	hardware_interface
	transmission_interface
	diagnostic_updater
)

kacanopen_install(kacanopen_roscontrol_node TRUE)
