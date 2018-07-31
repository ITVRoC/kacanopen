#ifndef KACANOPEN_HARDWARE_H
#define KACANOPEN_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include "kacanopen_manager.h"
#include "master.h"
#include "bridge.h"

namespace kaco
{

class KaCanopenHardware : public hardware_interface::RobotHW
{
public:
  KaCanopenHardware(Master* master, ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::vector<std::string>& motor_names);
  bool init();
  void read();
  void write();
  void updateDiagnostics();
private:
  hardware_interface::ActuatorStateInterface asi;
  hardware_interface::VelocityActuatorInterface avi;
  hardware_interface::PositionActuatorInterface api;

  KaCanopenManager manager_;

  transmission_interface::RobotTransmissions robot_transmissions;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader;
};
}

#endif // KACANOPEN_HARDWARE_H
