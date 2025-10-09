cmake_minimum_required(VERSION 3.8)
project(kacanopen)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### settings ###

include(CMakeSettings.txt)

## Driver begin ##

if (DEFINED CAN_DRIVER_NAME)
	message(WARNING "CAN_DRIVER_NAME is deprecated. Use DRIVER instead. Overwriting DRIVER with CAN_DRIVER_NAME (${CAN_DRIVER_NAME}) now...")
	set(DRIVER "${CAN_DRIVER_NAME}")
endif()

foreach(BUILD_DRIVER ${BUILD_DRIVERS})
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND SUPPORTED_DRIVERS "${BUILD_DRIVER}" supported)
	if (${supported} EQUAL -1)
		message(FATAL_ERROR "Driver ${BUILD_DRIVER} does not exist, but you've specified it in BUILD_DRIVERS.")
	endif()
endforeach()

list(FIND SUPPORTED_DRIVERS "${DRIVER}" driver_supported)
if (${driver_supported} EQUAL -1)
	message(FATAL_ERROR "Driver ${DRIVER} does not exist, but you've set it as DRIVER.")
endif()

set(DRIVER_LINK_NAME can_${DRIVER})
message(STATUS "Using CAN driver ${DRIVER_LINK_NAME}")

if(${BUILD_ALL_DRIVERS})
	message(STATUS "Building all available drivers.")
endif()

## Driver end ##

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
	message(STATUS "You are in release mode.")
else()
	if(${EXHAUSTIVE_DEBUGGING})
		message(STATUS "Enabled exhaustive debugging.")
		add_definitions("-DEXHAUSTIVE_DEBUGGING")
	else()
		message(STATUS "Enabled debugging.")
	endif()
endif()



### constants ###

add_definitions("-DSDO_RESPONSE_TIMEOUT_MS=${SDO_RESPONSE_TIMEOUT_MS}")
message(STATUS "SDO response timeout is set to ${SDO_RESPONSE_TIMEOUT_MS}ms")

add_definitions("-DBUSNAME=\"${BUSNAME}\"")
message(STATUS "Busname is set to ${BUSNAME} (only used by examples)")

add_definitions("-DBAUDRATE=${BAUDRATE}")
message(STATUS "Baudrate is set to ${BAUDRATE} (only used by examples)")

add_definitions("-DCONSECUTIVE_SEND_PAUSE_MS=${CONSECUTIVE_SEND_PAUSE_MS}")
message(STATUS "Pause between two consecutively sent CAN frames is set to ${CONSECUTIVE_SEND_PAUSE_MS}ms")


### dependencies ###

find_package(Threads)
find_package(Boost 1.46.1 COMPONENTS system filesystem REQUIRED)

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
	message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
message(STATUS "Found Boost system library: ${Boost_SYSTEM_LIBRARY}")
message(STATUS "Found Boost filesystem library: ${Boost_FILESYSTEM_LIBRARY}")
message(STATUS "Found Threads library: ${CMAKE_THREAD_LIBS_INIT}")


### ROS 2 ###

if (NOT(${NO_ROS}))
	# Find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(std_msgs REQUIRED)
	find_package(sensor_msgs REQUIRED)
	find_package(std_srvs REQUIRED)
	find_package(controller_manager REQUIRED)
	find_package(hardware_interface REQUIRED)
	find_package(transmission_interface REQUIRED)
	find_package(diagnostic_updater REQUIRED)
	find_package(rosidl_default_generators REQUIRED)

	# Generate services
	rosidl_generate_interfaces(${PROJECT_NAME}
		"ros_bridge/srv/Mode.srv"
		"ros_bridge/srv/Action.srv"
		DEPENDENCIES std_msgs
	)

endif()


### includes ###

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_LIST_DIR}/core/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/master/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/ros_bridge/include)


### Installation ###

# argument 1: targets
# argument 2: if true, install ${CMAKE_CURRENT_LIST_DIR}/include/
# call this from subproject's CMake files
macro(kacanopen_install)
	message(STATUS "Configuring installation for target(s) ${ARGV0}")
	if (${NO_ROS})
		install(TARGETS ${ARGV0}
			LIBRARY DESTINATION lib
			RUNTIME DESTINATION bin
		)
		if (${ARGV1})
			install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
				DESTINATION include/kacanopen
			)
		endif()
	else()
		install(TARGETS ${ARGV0}
			DESTINATION lib/${PROJECT_NAME}
		)
		if (${ARGV1})
			install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
				DESTINATION include/${PROJECT_NAME}
			)
		endif()
	endif()
endmacro()


# Install EDS library
message(STATUS "Configuring installation for EDS library")
add_definitions("-DSHARE_SOURCE_PATH=\"${CMAKE_CURRENT_LIST_DIR}/master/share\"")
add_definitions("-DSHARE_INSTALLED_PATH=\"${CMAKE_INSTALL_PREFIX}/share/kacanopen\"")
message(STATUS "SHARE_SOURCE_PATH set to ${CMAKE_CURRENT_LIST_DIR}/share.")
message(STATUS "SHARE_INSTALLED_PATH set to ${CMAKE_INSTALL_PREFIX}/share/kacanopen.")
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/master/share/
	DESTINATION share/kacanopen
)

### Source code ###

if (NOT(${NO_ROS}))
	add_subdirectory(ros_bridge)
endif()

add_subdirectory(drivers_lgpl)
add_subdirectory(drivers)
add_subdirectory(core)
add_subdirectory(master)
add_subdirectory(espeleo)
add_subdirectory(tools)

# ROS 2 ament export
if (NOT(${NO_ROS}))
	# Install launch files
	install(DIRECTORY launch/
		DESTINATION share/${PROJECT_NAME}/launch
		FILES_MATCHING PATTERN "*.py"
	)

	# Export dependencies
	ament_export_dependencies(
		rclcpp
		std_msgs
		sensor_msgs
		std_srvs
		controller_manager
		hardware_interface
		transmission_interface
		diagnostic_updater
	)
	
	# Export include directories
	ament_export_include_directories(
		core/include
		master/include
		ros_bridge/include
	)
	
	# Export libraries
	ament_export_libraries(
		kacanopen_core
		kacanopen_master
		kacanopen_ros_bridge
	)
	
	# Generate and export targets
	rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
	
	ament_package()
endif()
