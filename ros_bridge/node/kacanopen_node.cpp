#include <rclcpp/rclcpp.hpp>
#include <kacanopen_hardware.h>
// #include <controller_manager/controller_manager.h> // TODO: ROS 2 controller manager conversion
#include <vector>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kacanopen_velocity_hardware");

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  std::string busname = "slcan0";
  node->declare_parameter("busname", busname);
  busname = node->get_parameter("busname").as_string();

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  std::string baudrate = "500K";
  node->declare_parameter("baudrate", baudrate);
  baudrate = node->get_parameter("baudrate").as_string();

  std::vector<std::string> motor_names;
  for(int i = 0; i < argc-1; ++i) {
    motor_names.push_back(argv[i+1]);
  }
  kaco::Master master;
  // ROS 2 hardware interface - now properly converted
  kaco::KaCanopenHardware robot(&master, node, node, motor_names);
  // TODO: controller_manager conversion to ros2_control needed
  // controller_manager::ControllerManager cm(&robot, node);

  if (!master.start(busname, baudrate)) {
    RCLCPP_ERROR(node->get_logger(), "Starting master failed.");
    return -1;
  }

  //master.core.nmt.reset_all_nodes();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  size_t num_devices_required = 1;
  while (master.num_devices()<num_devices_required && rclcpp::ok()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
    RCLCPP_INFO(node->get_logger(), "Trying to discover more nodes via NMT Node Guarding...");
    master.core.nmt.discover_nodes();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Initialize and configure the hardware interface
  if (!robot.init()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize hardware interface");
    return -1;
  }
  
  if (!robot.configure()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure hardware interface");
    return -1;
  }
  
  if (!robot.activate()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate hardware interface");
    return -1;
  }
  
  RCLCPP_INFO(node->get_logger(), "Hardware interface initialized, configured, and activated successfully");
  RCLCPP_INFO_STREAM(node->get_logger(), "Managing " << robot.getNumMotors() << " motors: " 
                     << [&]() {
                       std::string names;
                       for (const auto& name : robot.getMotorNames()) {
                         names += name + " ";
                       }
                       return names;
                     }());
  RCLCPP_INFO(node->get_logger(), "KaCanOpen ROS 2 hardware interface ready");
  
  // Spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();

}
