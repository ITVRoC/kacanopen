#include "kacanopen_hardware.h"

namespace kaco
{
KaCanopenHardware::KaCanopenHardware(Master* master, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<rclcpp::Node> pnode, const std::vector<std::string>& motor_names)
  : master_(master), node_(node), pnode_(pnode), motor_names_(motor_names), manager_(master, node, pnode, motor_names),
    configured_(false), activated_(false)
{
  RCLCPP_INFO(node_->get_logger(), "KaCanopenHardware initialized for ROS 2");
  
  // Initialize state vectors
  size_t num_motors = motor_names_.size();
  position_states_.resize(num_motors, 0.0);
  velocity_states_.resize(num_motors, 0.0);
  effort_states_.resize(num_motors, 0.0);
  position_commands_.resize(num_motors, 0.0);
  velocity_commands_.resize(num_motors, 0.0);
  effort_commands_.resize(num_motors, 0.0);
  
  RCLCPP_INFO_STREAM(node_->get_logger(), "Initialized hardware interface for " << num_motors << " motors");
}

bool KaCanopenHardware::init()
{
  return manager_.init();
}

bool KaCanopenHardware::configure()
{
  if (configured_) {
    RCLCPP_WARN(node_->get_logger(), "Hardware already configured");
    return true;
  }
  
  bool success = manager_.configure();
  if (success) {
    configured_ = true;
    RCLCPP_INFO(node_->get_logger(), "Hardware configured successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure hardware");
  }
  return success;
}

bool KaCanopenHardware::activate()
{
  if (!configured_) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot activate unconfigured hardware");
    return false;
  }
  
  if (activated_) {
    RCLCPP_WARN(node_->get_logger(), "Hardware already activated");
    return true;
  }
  
  bool success = manager_.activate();
  if (success) {
    activated_ = true;
    RCLCPP_INFO(node_->get_logger(), "Hardware activated successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to activate hardware");
  }
  return success;
}

bool KaCanopenHardware::deactivate()
{
  if (!activated_) {
    RCLCPP_WARN(node_->get_logger(), "Hardware already deactivated");
    return true;
  }
  
  bool success = manager_.deactivate();
  if (success) {
    activated_ = false;
    RCLCPP_INFO(node_->get_logger(), "Hardware deactivated successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to deactivate hardware");
  }
  return success;
}

void KaCanopenHardware::updateDiagnostics()
{
  manager_.updateDiagnostics();
}

void KaCanopenHardware::read()
{
  if (!activated_) {
    return;
  }
  
  manager_.read();
  
  // Update state vectors from manager
  position_states_ = manager_.getPositions();
  velocity_states_ = manager_.getVelocities();
  effort_states_ = manager_.getEfforts();
}

void KaCanopenHardware::write()
{
  if (!activated_) {
    return;
  }
  
  // Send commands to manager
  manager_.setVelocityCommands(velocity_commands_);
  manager_.setPositionCommands(position_commands_);
  manager_.setEffortCommands(effort_commands_);
  
  manager_.write();
}

// State and command access methods
std::vector<double> KaCanopenHardware::getPositions() const
{
  return position_states_;
}

std::vector<double> KaCanopenHardware::getVelocities() const
{
  return velocity_states_;
}

std::vector<double> KaCanopenHardware::getEfforts() const
{
  return effort_states_;
}

void KaCanopenHardware::setVelocityCommands(const std::vector<double>& velocities)
{
  if (velocities.size() != velocity_commands_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Velocity command size mismatch: expected " 
                        << velocity_commands_.size() << ", got " << velocities.size());
    return;
  }
  velocity_commands_ = velocities;
}

void KaCanopenHardware::setPositionCommands(const std::vector<double>& positions)
{
  if (positions.size() != position_commands_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Position command size mismatch: expected " 
                        << position_commands_.size() << ", got " << positions.size());
    return;
  }
  position_commands_ = positions;
}

void KaCanopenHardware::setEffortCommands(const std::vector<double>& efforts)
{
  if (efforts.size() != effort_commands_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Effort command size mismatch: expected " 
                        << effort_commands_.size() << ", got " << efforts.size());
    return;
  }
  effort_commands_ = efforts;
}

}
