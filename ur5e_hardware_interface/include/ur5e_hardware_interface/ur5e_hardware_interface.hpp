#ifndef URBOT_HARDWARE_INTERFACE__URBOT_HARDWARE_INTERFACE_HPP_
#define URBOT_HARDWARE_INTERFACE__URBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <sensor_msgs/JointState.h> 
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

namespace ur5e_hardware_interface
{
class UR5eHardwareInterface : public hardware_interface::RobotHW
{
public:
  // Initialize the hardware interface
  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh);

  // Callback function to update joint state data
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

  // Read current robot states from hardware
  bool read(const ros::Time time, const ros::Duration period);

  // Write commands to hardware
  bool write(const ros::Time time, const ros::Duration period);

private:
  // Interfaces for joint state and position command
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface position_command_interface_;
  
  // ROS subscriber and publisher for joint states and commands
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher mujoco_commands_pub;
  
  // Data vectors for holding hardware state and command values
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  // Latest received joint state data
  std::vector<double> latest_position_;
  std::vector<double> latest_velocity_;
  std::vector<double> latest_effort_;

  // Names of the robot's joints
  std::vector<std::string> joint_names_;
};

}  // namespace ur5e_hardware_interface

#endif  // URBOT_HARDWARE_INTERFACE__URBOT_HARDWARE_INTERFACE_HPP_
