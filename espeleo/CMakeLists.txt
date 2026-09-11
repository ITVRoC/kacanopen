cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring espeleo")

file(GLOB TARGETS_SRC
	"*.cpp"
	"test/*.cpp"
)

if (NOT(${NO_ROS}))
	# Manually specify ROS source files - now converted to ROS 2
	set(TARGETS_SRC_ROS
		"src/espeleo_bridge.cpp"
		# Temporarily exclude: "src/callback_test_problem.cpp" - needs test class implementations
	)
	list(APPEND TARGETS_SRC ${TARGETS_SRC_ROS})
endif()

foreach(TARGET_SRC ${TARGETS_SRC})

	get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
	set(TARGET_NAME "kacanopen_${TARGET_NAME}")
	add_executable(${TARGET_NAME} ${TARGET_SRC})
	set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 17)
	target_link_libraries(${TARGET_NAME} kacanopen_core)
	target_link_libraries(${TARGET_NAME} kacanopen_master)
	
	if (NOT(${NO_ROS}))
		target_link_libraries(${TARGET_NAME} kacanopen_ros_bridge)
		# ROS 2 dependencies
		ament_target_dependencies(${TARGET_NAME}
			rclcpp
			std_msgs
			sensor_msgs
			std_srvs
		)
	endif()
	
	list(APPEND KACANOPEN_TARGET_NAMES ${TARGET_NAME})

endforeach(TARGET_SRC ${TARGETS_SRC})

# Install
#if (${INSTALL_EXAMPLES})
#	kacanopen_install("${KACANOPEN_TARGET_NAMES}" FALSE)
#endif()

install(TARGETS ${KACANOPEN_TARGET_NAMES}
    DESTINATION lib/${PROJECT_NAME}
)
