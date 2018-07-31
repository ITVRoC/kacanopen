#ifndef KACANOPEN_MOTOR_H
#define KACANOPEN_MOTOR_H

#include <ros/ros.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "master.h"
#include "device.h"

namespace kaco
{
class KaCanopenMotor
{
public:
  KaCanopenMotor(Master* master, const std::string& name,
           ros::NodeHandle& nh,
           ros::NodeHandle& config_nh,
           hardware_interface::ActuatorStateInterface& asi,
           hardware_interface::VelocityActuatorInterface& avi,
           hardware_interface::PositionActuatorInterface& api);
  std::string actuatorName() const;
  std::string name() const;

  bool init();
  void read();
  void write();
  void updateDiagnostics();

private:
  double pos_to_rad(int32_t pos) const;
  bool valid_;
  std::string name_;
  std::string actuator_name_;
  ros::NodeHandle nh_;
  ros::NodeHandle config_nh_;
  std::string device_serial_;
  size_t device_id_;
  bool use_serial_number_;
  int encoder_min_;
  int encoder_max_;

  Master *master_;
  Device *device_;

  double position_;
  double velocity_;
  double effort_;

  double position_cmd_;
  double velocity_cmd_;
};
}

#endif // KACANOPEN_MOTOR_H
