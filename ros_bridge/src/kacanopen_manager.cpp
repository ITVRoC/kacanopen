#include "kacanopen_manager.h"

namespace kaco
{
KaCanopenManager::KaCanopenManager(Master *master, 
       std::shared_ptr<rclcpp::Node> node, 
       std::shared_ptr<rclcpp::Node> pnode,
       const std::vector<std::string>& motor_names)
  : master_(master), node_(node), pnode_(pnode), motor_names_(motor_names),
    configured_(false), activated_(false)
{
  for (const auto& m : motor_names)
  {
    // Create a sub-node for each motor configuration
    auto config_node = std::make_shared<rclcpp::Node>(std::string(pnode->get_name()) + "_" + m);
    motors_.push_back(std::make_shared<KaCanopenMotor>(master, m, node, config_node));
  }
  
  RCLCPP_INFO_STREAM(node_->get_logger(), "KaCanopenManager initialized with " << motors_.size() << " motors");
}

bool KaCanopenManager::init()
{
  bool success = true;
  for (const auto& m : motors_)
  {
    if (!m->init())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not configure motor: " << m->name());
      success = false;
    }
  }
  return success;
}

bool KaCanopenManager::configure()
{
  if (configured_) {
    RCLCPP_WARN(node_->get_logger(), "Manager already configured");
    return true;
  }
  
  bool success = init();
  if (success) {
    configured_ = true;
    RCLCPP_INFO(node_->get_logger(), "Manager configured successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure manager");
  }
  return success;
}

bool KaCanopenManager::activate()
{
  if (!configured_) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot activate unconfigured manager");
    return false;
  }
  
  if (activated_) {
    RCLCPP_WARN(node_->get_logger(), "Manager already activated");
    return true;
  }
  
  activated_ = true;
  RCLCPP_INFO(node_->get_logger(), "Manager activated successfully");
  return true;
}

bool KaCanopenManager::deactivate()
{
  if (!activated_) {
    RCLCPP_WARN(node_->get_logger(), "Manager already deactivated");
    return true;
  }
  
  activated_ = false;
  RCLCPP_INFO(node_->get_logger(), "Manager deactivated successfully");
  return true;
}

void KaCanopenManager::updateDiagnostics()
{
  for (const auto& m : motors_)
    m->updateDiagnostics();
}

void KaCanopenManager::read()
{
  if (!activated_) {
    return;
  }
  
  for (const auto& m : motors_)
    m->read();
}

void KaCanopenManager::write()
{
  if (!activated_) {
    return;
  }
  
  for (const auto& m : motors_)
    m->write();
}

// State and command interface implementation
std::vector<double> KaCanopenManager::getPositions() const
{
  std::vector<double> positions;
  positions.reserve(motors_.size());
  
  for (const auto& motor : motors_) {
    positions.push_back(motor->getPosition());
  }
  
  return positions;
}

std::vector<double> KaCanopenManager::getVelocities() const
{
  std::vector<double> velocities;
  velocities.reserve(motors_.size());
  
  for (const auto& motor : motors_) {
    velocities.push_back(motor->getVelocity());
  }
  
  return velocities;
}

std::vector<double> KaCanopenManager::getEfforts() const
{
  std::vector<double> efforts;
  efforts.reserve(motors_.size());
  
  for (const auto& motor : motors_) {
    efforts.push_back(motor->getEffort());
  }
  
  return efforts;
}

void KaCanopenManager::setVelocityCommands(const std::vector<double>& velocities)
{
  if (velocities.size() != motors_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Velocity command size mismatch: expected " 
                        << motors_.size() << ", got " << velocities.size());
    return;
  }
  
  for (size_t i = 0; i < motors_.size(); ++i) {
    motors_[i]->setVelocityCommand(velocities[i]);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Setting velocity " << velocities[i] 
                        << " for motor " << i);
  }
}

void KaCanopenManager::setPositionCommands(const std::vector<double>& positions)
{
  if (positions.size() != motors_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Position command size mismatch: expected " 
                        << motors_.size() << ", got " << positions.size());
    return;
  }
  
  for (size_t i = 0; i < motors_.size(); ++i) {
    motors_[i]->setPositionCommand(positions[i]);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Setting position " << positions[i] 
                        << " for motor " << i);
  }
}

void KaCanopenManager::setEffortCommands(const std::vector<double>& efforts)
{
  if (efforts.size() != motors_.size()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Effort command size mismatch: expected " 
                        << motors_.size() << ", got " << efforts.size());
    return;
  }
  
  for (size_t i = 0; i < motors_.size(); ++i) {
    motors_[i]->setEffortCommand(efforts[i]);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Setting effort " << efforts[i] 
                        << " for motor " << i);
  }
}
}
