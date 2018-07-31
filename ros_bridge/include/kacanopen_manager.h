#ifndef KACANOPEN_MANAGER_H
#define KACANOPEN_MANAGER_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "kacanopen_motor.h"
#include "master.h"

namespace kaco
{
class KaCanopenManager
{
public:
  KaCanopenManager(Master* master, hardware_interface::ActuatorStateInterface& asi,
        hardware_interface::VelocityActuatorInterface& avi,
        hardware_interface::PositionActuatorInterface& api,
        ros::NodeHandle& nh, ros::NodeHandle& pnh,
        const std::vector<std::string>& motor_names);
  bool init();
  void read();
  void write();
  void updateDiagnostics();
  std::vector<std::shared_ptr<KaCanopenMotor> > motors() { return motors_; }
private:
  std::vector<std::shared_ptr<KaCanopenMotor> > motors_;

  hardware_interface::ActuatorStateInterface* asi_;
  hardware_interface::VelocityActuatorInterface* avi_;
  hardware_interface::PositionActuatorInterface* api_;
};
}

#endif // KACANOPEN_MANAGER_H
