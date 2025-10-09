#ifndef KACANOPEN_MANAGER_H
#define KACANOPEN_MANAGER_H

#include <rclcpp/rclcpp.hpp>
// ROS 2 hardware interface includes
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "kacanopen_motor.h"
#include "master.h"

namespace kaco
{
class KaCanopenManager
{
public:
  KaCanopenManager(Master* master, 
        std::shared_ptr<rclcpp::Node> node, 
        std::shared_ptr<rclcpp::Node> pnode,
        const std::vector<std::string>& motor_names);
  ~KaCanopenManager() = default;
  
  bool init();
  void read();
  void write();
  void updateDiagnostics();
  
  // Motor access and management
  std::vector<std::shared_ptr<KaCanopenMotor>> motors() { return motors_; }
  const std::vector<std::shared_ptr<KaCanopenMotor>>& motors() const { return motors_; }
  size_t getNumMotors() const { return motors_.size(); }
  
  // State and command interface for ROS 2 hardware interface
  std::vector<double> getPositions() const;
  std::vector<double> getVelocities() const;
  std::vector<double> getEfforts() const;
  void setVelocityCommands(const std::vector<double>& velocities);
  void setPositionCommands(const std::vector<double>& positions);
  void setEffortCommands(const std::vector<double>& efforts);
  
  // Configuration and lifecycle
  bool configure();
  bool activate();
  bool deactivate();

private:
  Master* master_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> pnode_;
  std::vector<std::string> motor_names_;
  std::vector<std::shared_ptr<KaCanopenMotor>> motors_;
  
  // State management
  bool configured_;
  bool activated_;
};
}

#endif // KACANOPEN_MANAGER_H
