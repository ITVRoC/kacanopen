#include "kacanopen_motor.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"
#include "sdo_error.h"

namespace kaco
{
KaCanopenMotor::KaCanopenMotor(Master *master, const std::string &name, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<rclcpp::Node> config_node)
  : valid_(true)
  , name_(name)
  , node_(node)
  , config_node_(config_node)
  , device_id_(0)
  , use_serial_number_(true)
  , encoder_min_(0)
  , encoder_max_(65535)
  , master_(master)
  , position_(0.0)
  , velocity_(0.0)
  , effort_(0.0)
  , position_cmd_(0.0)
  , velocity_cmd_(0.0)
  , effort_cmd_(0.0)
{
  // TODO: ROS 2 parameter conversion needed
  config_node_->declare_parameter("actuator_name", "");
  if (!config_node_->get_parameter("actuator_name", actuator_name_) || actuator_name_.empty())
  {
    RCLCPP_ERROR(config_node_->get_logger(), "You must specify an actuator name");
    valid_ = false;
  }

  int device_id = 0;
  config_node_->declare_parameter("serial_number", "");
  config_node_->declare_parameter("device_id", 0);
  
  std::string temp_serial;
  bool has_serial = config_node_->get_parameter("serial_number", temp_serial) && !temp_serial.empty();
  bool has_device_id = config_node_->get_parameter("device_id", device_id) && device_id != 0;
  
  if (!has_serial && !has_device_id)
  {
    RCLCPP_ERROR(config_node_->get_logger(), "You must specify a serial number or a device_id");
    valid_ = false;
  }
  else if (has_serial)
  {
    device_serial_ = temp_serial;
    use_serial_number_ = true;
  }
  else if (has_device_id)
  {
    if (device_id < 0)
    {
      RCLCPP_ERROR(config_node_->get_logger(), "Device id must be >= 0");
      valid_ = false;
    }
    else
    {
      device_id_ = static_cast<size_t>(device_id);
    }
    use_serial_number_ = false;
  }
  // ROS 2 parameter conversion
  config_node_->declare_parameter("sensor.encoder_min", encoder_min_);
  config_node_->declare_parameter("sensor.encoder_max", encoder_max_);
  encoder_min_ = config_node_->get_parameter("sensor.encoder_min").as_int();
  encoder_max_ = config_node_->get_parameter("sensor.encoder_max").as_int();
  
  if (encoder_min_ == encoder_max_)
    throw std::invalid_argument("encoder_max must be different from encoder_min");

  RCLCPP_INFO_STREAM(node_->get_logger(), actuator_name_);
  
  // TODO: ROS 2 hardware interface - these will be handled by ros2_control framework
  // hardware_interface::ActuatorStateHandle state_handle(actuator_name_, &position_, &velocity_, &effort_);
  // asi.registerHandle(state_handle);
  // hardware_interface::ActuatorHandle position_handle(state_handle, &position_cmd_);
  // api.registerHandle(position_handle);
  // hardware_interface::ActuatorHandle velocity_handle(state_handle, &velocity_cmd_);
  // avi.registerHandle(velocity_handle);

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
    RCLCPP_ERROR(node_->get_logger(), "Not implemented!");
    return false;
    // Loop stuff
  }
  else
    device_ = &master_->get_device(device_id_);
  device_->start();

  device_->load_dictionary_from_library();

  const auto profile = device_->get_device_profile_number();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID "
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


    RCLCPP_INFO(node_->get_logger(), "Set velocity mode");
    device_->set_entry("modes_of_operation", device_->get_constant("profile_velocity_mode"));

    RCLCPP_INFO(node_->get_logger(), "Enable operation");
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
    device_->add_transmit_pdo_mapping(0x200 + device_id_, mapping_200);
    //device.add_transmit_pdo_mapping(0x300 + device_id_, mapping_300);
    device_->add_transmit_pdo_mapping(0x400 + device_id_, mapping_400);
    //device.add_transmit_pdo_mapping(0x500 + device_id_, mapping_500);


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
    RCLCPP_INFO_STREAM(node_->get_logger(), "Velocity " << velocity_ << ", pos " << position_);
  }
  catch (const sdo_error& error)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception in " << __FUNCTION__ << ": " << error.what());
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
    RCLCPP_INFO_STREAM(node_->get_logger(), "Velocity command " << velocity_cmd_);
  } catch (const sdo_error& error) {
    // TODO: only catch timeouts?
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception in " << __FUNCTION__ << ": " << error.what());
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
