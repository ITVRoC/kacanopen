#include "kacanopen_manager.h"

namespace kaco
{
KaCanopenManager::KaCanopenManager(Master *master, hardware_interface::ActuatorStateInterface& asi,
       hardware_interface::VelocityActuatorInterface& avi,
       hardware_interface::PositionActuatorInterface& api,
       ros::NodeHandle& nh, ros::NodeHandle& pnh,
       const std::vector<std::string>& motor_names)
  : asi_(&asi), avi_(&avi), api_(&api)
{
  for (const auto& m : motor_names)
  {
    ros::NodeHandle config_nh(pnh, m);
    motors_.push_back(std::make_shared<KaCanopenMotor>(master, m, nh, config_nh, asi, avi, api));
  }
}

bool KaCanopenManager::init()
{
  bool success = true;
  for (const auto& m : motors_)
  {
    if (!m->init())
    {
      ROS_ERROR_STREAM("Could not configure motor: " << m->name());
      success = false;
    }
  }
  return success;
}

void KaCanopenManager::updateDiagnostics()
{
  for (const auto& m : motors_)
    m->updateDiagnostics();
}

void KaCanopenManager::read()
{
  for (const auto& m : motors_)
    m->read();
}

void KaCanopenManager::write()
{
  for (const auto& m : motors_)
    m->write();
}
}
