#ifndef KACANOPEN_HARDWARE_H
#define KACANOPEN_HARDWARE_H

#include <rclcpp/rclcpp.hpp>
// ROS 2 hardware interface includes
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include "kacanopen_manager.h"
#include "master.h"
#include "bridge.h"

namespace kaco
{

// ROS 2 hardware interface - fully functional implementation
class KaCanopenHardware
{
public:
  KaCanopenHardware(Master* master, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<rclcpp::Node> pnode, const std::vector<std::string>& motor_names);
  ~KaCanopenHardware() = default;
  
  bool init();
  void read();
  void write();
  void updateDiagnostics();
  
  // ROS 2 hardware interface methods
  bool configure();
  bool activate();
  bool deactivate();
  
  // State and command access
  std::vector<double> getPositions() const;
  std::vector<double> getVelocities() const;
  std::vector<double> getEfforts() const;
  void setVelocityCommands(const std::vector<double>& velocities);
  void setPositionCommands(const std::vector<double>& positions);
  void setEffortCommands(const std::vector<double>& efforts);
  
  // Motor management
  size_t getNumMotors() const { return motor_names_.size(); }
  const std::vector<std::string>& getMotorNames() const { return motor_names_; }

private:
  Master* master_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> pnode_;
  std::vector<std::string> motor_names_;
  KaCanopenManager manager_;
  
  // Hardware interface state
  bool configured_;
  bool activated_;
  
  // State vectors for ROS 2 hardware interface
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> effort_commands_;
};
}

#endif // KACANOPEN_HARDWARE_H
