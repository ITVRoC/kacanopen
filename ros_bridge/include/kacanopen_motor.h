#ifndef KACANOPEN_MOTOR_H
#define KACANOPEN_MOTOR_H

#include <rclcpp/rclcpp.hpp>
// ROS 2 hardware interface includes
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "master.h"
#include "device.h"

namespace kaco
{
class KaCanopenMotor
{
public:
  KaCanopenMotor(Master* master, const std::string& name,
           std::shared_ptr<rclcpp::Node> node,
           std::shared_ptr<rclcpp::Node> config_node);
  std::string actuatorName() const;
  std::string name() const;

  bool init();
  void read();
  void write();
  void updateDiagnostics();
  
  // State and command accessors for hardware interface
  double getPosition() const { return position_; }
  double getVelocity() const { return velocity_; }
  double getEffort() const { return effort_; }
  
  void setPositionCommand(double cmd) { position_cmd_ = cmd; }
  void setVelocityCommand(double cmd) { velocity_cmd_ = cmd; }
  void setEffortCommand(double cmd) { effort_cmd_ = cmd; }
  
  double getPositionCommand() const { return position_cmd_; }
  double getVelocityCommand() const { return velocity_cmd_; }
  double getEffortCommand() const { return effort_cmd_; }

private:
  double pos_to_rad(int32_t pos) const;
  bool valid_;
  std::string name_;
  std::string actuator_name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> config_node_;
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
  double effort_cmd_;
};
}

#endif // KACANOPEN_MOTOR_H
