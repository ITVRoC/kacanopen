#include "kacanopen_motor.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"
#include "sdo_error.h"

namespace kaco
{
KaCanopenMotor::KaCanopenMotor(Master *master, const std::string &name, ros::NodeHandle &nh, ros::NodeHandle &config_nh, hardware_interface::ActuatorStateInterface &asi, hardware_interface::VelocityActuatorInterface &avi, hardware_interface::PositionActuatorInterface &api)
  : valid_(true)
  , name_(name)
  , nh_(nh)
  , config_nh_(config_nh)
  , device_id_(0)
  , use_serial_number_(true)
  , encoder_min_(0)
  , encoder_max_(65535)
  , master_(master)
{
  if (!config_nh_.getParam("actuator_name", actuator_name_))
  {
    ROS_ERROR("You must specify an actuator name");
    valid_ = false;
  }

  int device_id = 0;
  if (!config_nh_.getParam("serial_number", device_serial_) && !config_nh_.getParam("device_id", device_id))
  {
    ROS_ERROR("You must specify a serial number or a port name");
    valid_ = false;
  }
  else if (config_nh_.getParam("serial_number", device_serial_))
  {
    use_serial_number_ = true;
  }
  else if (config_nh_.getParam("device_id", device_id))
  {
    if (device_id < 0)
    {
      ROS_ERROR("Device id must be >= 0");
      valid_ = false;
    }
    else
    {
      device_id_ = static_cast<size_t>(device_id);
    }
    use_serial_number_ = false;
  }
  config_nh_.param("sensor/encoder_min", encoder_min_, encoder_min_);
  config_nh_.param("sensor/encoder_max", encoder_max_, encoder_max_);
  if (encoder_min_ == encoder_max_)
    throw std::invalid_argument("encoder_max must be different from encoder_min");

  ROS_INFO_STREAM(actuator_name_);
  hardware_interface::ActuatorStateHandle state_handle(actuator_name_, &position_, &velocity_, &effort_);
  asi.registerHandle(state_handle);

  hardware_interface::ActuatorHandle position_handle(state_handle, &position_cmd_);
  api.registerHandle(position_handle);
  hardware_interface::ActuatorHandle velocity_handle(state_handle, &velocity_cmd_);
  avi.registerHandle(velocity_handle);

  /*
  diagnostic_updater_.setHardwareID(serial_number_str);
  std::stringstream motor_diagnostic_name_ss;
  motor_diagnostic_name_ss << name << ": "
                           << "Motor";
  diagnostic_updater_.add(motor_diagnostic_name_ss.str(), boost::bind(&Epos::buildMotorStatus, this, _1));
  std::stringstream motor_output_diagnostic_name_ss;
  motor_output_diagnostic_name_ss << name << ": "
                                  << "Motor Output";
  diagnostic_updater_.add(motor_output_diagnostic_name_ss.str(), boost::bind(&Epos::buildMotorOutputStatus, this, _1));
  */
}

std::string KaCanopenMotor::actuatorName() const
{
  return actuator_name_;
}

std::string KaCanopenMotor::name() const
{
  return name_;
}

bool KaCanopenMotor::init()
{
  if (use_serial_number_)
  {
    ROS_ERROR("Not implemented!");
    return false;
    // Loop stuff
  }
  else
    device_ = &master_->get_device(device_id_);
  device_->start();

  device_->load_dictionary_from_library();

  const auto profile = device_->get_device_profile_number();
  ROS_INFO_STREAM("Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID "
                  <<static_cast<int>(device_->get_node_id())<<": "<<device_->get_entry("manufacturer_device_name"));
  bool found = false;

  if (profile==401) {

    found = true;

    // TODO: we should determine the number of input / output bytes fiŕst.

    // map PDOs (optional)
    device_->add_receive_pdo_mapping(0x188, "Read input 8-bit/Digital Inputs 1-8", 0); // offest 0
    device_->add_receive_pdo_mapping(0x188, "Read input 8-bit/Digital Inputs 9-16", 1); // offset 1



    // set some output (optional)
    device_->set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t) 0xFF);
    return true;

  } else if (profile==402) {

    found = true;


    ROS_INFO("Set velocity mode");
    device_->set_entry("modes_of_operation", device_->get_constant("profile_velocity_mode"));

    ROS_INFO("Enable operation");
    device_->execute("enable_operation");

    //PDOs for General Control word (0x200 + Device id)
    //PDOs for Profile Position (0x300 + Device id)
    //PDOs for Profile Velocity (0x400 + Device id)
    //PDOs for Profile Torque (0x500 + Device id)
    std::vector<kaco::Mapping> mapping_200, mapping_300, mapping_400, mapping_500;
    kaco::Mapping mapping_cw, mapping_target_pos, mapping_target_vel, mapping_target_torq;
    mapping_cw.entry_name = "Controlword";
    mapping_cw.offset = 0;
    mapping_200.push_back(mapping_cw);
    mapping_300.push_back(mapping_cw);
    mapping_400.push_back(mapping_cw);
    mapping_500.push_back(mapping_cw);
    mapping_target_pos.entry_name = "Target Position";
    mapping_target_pos.offset = 2;
    mapping_300.push_back(mapping_target_pos);
    mapping_target_vel.entry_name = "Target Velocity";
    mapping_target_vel.offset = 2;
    mapping_400.push_back(mapping_target_vel);
    mapping_target_torq.entry_name = "Target Torque";
    mapping_target_torq.offset = 2;
    mapping_500.push_back(mapping_target_torq);
    device_->add_transmit_pdo_mapping(0x27F, mapping_200);
    //device.add_transmit_pdo_mapping(0x37F, mapping_300);
    device_->add_transmit_pdo_mapping(0x47F, mapping_400);
    //device.add_transmit_pdo_mapping(0x57F, mapping_500);


    // startup sequence
    device_->set_entry("Target Velocity",static_cast<int32_t>(0));
    device_->set_entry("Controlword", static_cast<uint16_t>(0x00));
    device_->set_entry("Controlword", static_cast<uint16_t>(0x06));
    device_->set_entry("Controlword", static_cast<uint16_t>(0x07));
    device_->set_entry("Controlword", static_cast<uint16_t>(0x1F));

    // recovery
    device_->set_entry("Controlword", static_cast<uint16_t>(0x1F));
    return true;
  }
  return false;
}

void KaCanopenMotor::read()
{
  try {
    const int32_t pos = device_->get_entry("Position actual value");
    const int32_t vel = device_->get_entry("Velocity actual value");
    position_ = pos_to_rad(pos);
    velocity_ = static_cast<double>(vel) * 2 * M_PI / (encoder_max_ - encoder_min_);
    ROS_INFO_STREAM("Velocity " << velocity_ << ", pos " << position_);
  }
  catch (const sdo_error& error)
  {
    ROS_ERROR_STREAM("Exception in " << __FUNCTION__ << ": " << error.what());
  }
  effort_ = 0;
}

void KaCanopenMotor::write()
{
  //device_->execute("set_target_position",static_cast<int32_t>(msg.velocity[0]));
  try {
    // vel_cmd_ is rad/s
    double ticks_per_sec = velocity_cmd_ * (encoder_max_ - encoder_min_) / (2 * M_PI);
    device_->set_entry("Target Velocity",static_cast<int32_t>(ticks_per_sec));
    device_->set_entry("Controlword", static_cast<uint16_t>(0x1F));
    ROS_INFO_STREAM("Velocity command " << velocity_cmd_);
  } catch (const sdo_error& error) {
    // TODO: only catch timeouts?
    ROS_ERROR_STREAM("Exception in " << __FUNCTION__ << ": " << error.what());
  }
}

void KaCanopenMotor::updateDiagnostics()
{

}

double kaco::KaCanopenMotor::pos_to_rad(int32_t pos) const
{
  return ((pos - encoder_min_) / (encoder_max_ - encoder_min_)) * 2 * M_PI;
}

}
