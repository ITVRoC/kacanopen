#include <ros/ros.h>
#include <ros/spinner.h>
#include <kacanopen_hardware.h>
#include <controller_manager/controller_manager.h>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kacanopen_velocity_hardware");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  std::string busname = "slcan0";
  pnh.param("busname", busname, busname);

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  std::string baudrate = "500K";
  pnh.param("baudrate", baudrate, baudrate);

  std::vector<std::string> motor_names;
  for(int i = 0; i < argc-1; ++i) {
    motor_names.push_back(argv[i+1]);
  }
  kaco::Master master;
  kaco::KaCanopenHardware robot(&master, nh, pnh, motor_names);
  controller_manager::ControllerManager cm(&robot, nh);

  if (!master.start(busname, baudrate)) {
    ROS_ERROR("Starting master failed.");
    return -1;
  }

  //master.core.nmt.reset_all_nodes();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  size_t num_devices_required = 1;
  while (master.num_devices()<num_devices_required && ros::ok()) {
    ROS_ERROR_STREAM("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
    ROS_INFO("Trying to discover more nodes via NMT Node Guarding...");
    master.core.nmt.discover_nodes();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Initializing Motors");
  if(!robot.init()) {
    ROS_FATAL("Failed to initialize motors");
    return 1;
  }
  ROS_INFO("Motors Initialized");

  ros::Rate controller_rate(50);
  ros::Time last = ros::Time::now();
  while (ros::ok()) {
    robot.read();
    ros::Time now = ros::Time::now();
    cm.update(now, now-last);
    robot.write();
    last = now;
    robot.updateDiagnostics();
    controller_rate.sleep();
  }

}
