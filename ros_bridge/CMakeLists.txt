cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring ros_bridge")

file(GLOB TARGET_SRC
	"src/*.cpp"
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

add_executable(kacanopen_roscontrol_node node/kacanopen_node.cpp)
set_property(TARGET kacanopen_roscontrol_node PROPERTY CXX_STANDARD 17)
target_link_libraries(kacanopen_roscontrol_node kacanopen_core kacanopen_ros_bridge kacanopen_master)

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
