#include "ros2_control_rover_hardware/rover_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace rover_hardware_
{
hardware_interface::CallbackReturn RoverHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
   RCLCPP_INFO(get_logger(), "1111111111111111111111111111");
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  // Imports sys_params_ from info_ (info_ origin: ros2_control.xacro or equivelent file in robot urfd)
  sys_params_.left_wheel_name          = info_.hardware_parameters["left_wheel_name"];
  sys_params_.right_wheel_name         = info_.hardware_parameters["right_wheel_name"];
  sys_params_.loop_rate                = std::stof(info_.hardware_parameters["loop_rate"]);  
  sys_params_.serial_device            = info_.hardware_parameters["device"];
  sys_params_.baud_rate                = std::stoi(info_.hardware_parameters["baud_rate"]);
  sys_params_.timeout_ms               = std::stoi(info_.hardware_parameters["timeout_ms"]);
  sys_params_.left_enc_counts_per_rev  = std::stoi(info_.hardware_parameters["left_enc_counts_per_rev"]);
  sys_params_.right_enc_counts_per_rev = std::stoi(info_.hardware_parameters["right_enc_counts_per_rev"]);


  left_wheel_.setup(sys_params_.left_wheel_name, sys_params_.left_enc_counts_per_rev);
  right_wheel_.setup(sys_params_.right_wheel_name, sys_params_.right_enc_counts_per_rev);


  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################



  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interface. 2 expected.", 
      joint.name.c_str(),joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
      joint.name.c_str(), joint.state_interfaces[0].name.c_str(),hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
      joint.name.c_str(), joint.state_interfaces[1].name.c_str(),hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn RoverHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  comms_.connect(sys_params_.serial_device,
    sys_params_.baud_rate,
    sys_params_.timeout_ms);
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################



  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn RoverHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




std::vector<hardware_interface::StateInterface::ConstSharedPtr> RoverHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  // Writes left wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

  // Writes right wheel's speed and position to state interfaces
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
  state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
    right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));


  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  return state_interfaces;
}








std::vector<hardware_interface::CommandInterface::SharedPtr> RoverHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));

  command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
    right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));


  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  return command_interfaces;
}



hardware_interface::CallbackReturn RoverHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RoverHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  comms_.disconnect();

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}





hardware_interface::return_type RoverHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
 
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################
  
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(left_wheel_.enc, right_wheel_.enc);

  double period_seconds = period.seconds();
  double pos_previous_left = left_wheel_.pos;
  double pos_previous_right = right_wheel_.pos;

  // Calculate the position and velocity of the wheels based on the encoder values
  // The formula is: pos = enc * rads_per_count, vel = (pos - pos_previous) / period_seconds
  left_wheel_.pos = left_wheel_.calcEncAngle();
  left_wheel_.vel = (left_wheel_.pos - pos_previous_left) / period_seconds;

  // Calculate the position and velocity of the right wheel in the same way
  right_wheel_.pos = right_wheel_.calcEncAngle();
  right_wheel_.vel = (right_wheel_.pos - pos_previous_right) / period_seconds;

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  // Calculate the motor speeds based on the command velocities and the loop rate
  // The formula is: speed = (cmd / rads_per_count) / loop_rate
  // where cmd is the command velocity in rad/s, rads_per_count is the conversion factor from counts to radians,
  int motor_left_speed = static_cast<int>((left_wheel_.cmd/left_wheel_.rads_per_count)/sys_params_.loop_rate);
  int motor_right_speed = static_cast<int>((right_wheel_.cmd/right_wheel_.rads_per_count)/sys_params_.loop_rate);

  // Set the motor values using the comms interface
  comms_.set_motor_values(motor_left_speed,motor_right_speed);

  //#################################################################################################################################
  //#################################################################################################################################
  //#################################################################################################################################

  return hardware_interface::return_type::OK;
}

}  // namespace rover_hardware_

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover_hardware_::RoverHardware, hardware_interface::SystemInterface)
