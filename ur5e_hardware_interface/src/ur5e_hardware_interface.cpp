#include "ur5e_hardware_interface/ur5e_hardware_interface.hpp"
#include <limits>
#include <vector>

namespace ur5e_hardware_interface
{

bool UR5eHardwareInterface::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle & robot_hw_nh)
{
  // Retrieve joint names from parameter server
  if (!robot_hw_nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter 'joint_names' on the parameter server.");
  }

  size_t num_joints = joint_names_.size();
  ROS_INFO_NAMED("UR5eHardwareInterface", "Found %zu joints.", num_joints);

  // Initialize state and command vectors for joint data
  hw_position_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  // Create ros_control interfaces for joint states and position commands
  for (size_t i = 0; i < num_joints; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_names_[i], &hw_position_states_[i], &hw_velocity_states_[i], &hw_effort_states_[i]));

    // Create joint position control interface
    position_command_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &hw_position_commands_[i]));

    velocity_command_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &hw_velocity_commands_[i]));
  }

  // Register interfaces with the hardware interface manager
  registerInterface(&joint_state_interface_);
  registerInterface(&position_command_interface_);
  // registerInterface(&velocity_command_interface_);


  // Print status message
  ROS_INFO_NAMED("UR5eHardwareInterface", "Starting...");

  // Set initial state positions to zero
  for (size_t i = 0; i < num_joints; ++i) {
    hw_position_states_[i] = 0.0;
    hw_position_commands_[i] = hw_position_states_[i];
    // hw_velocity_states_[i] = 0.0;
    // hw_velocity_commands_[i] = hw_position_states_[i];
  }

  // Subscribe to the joint state topic and advertise commands
  joint_state_subscriber_ = robot_hw_nh.subscribe("/mujoco_joint_state", 10, &UR5eHardwareInterface::jointStateCallback, this);
  mujoco_commands_pub = robot_hw_nh.advertise<sensor_msgs::JointState>("mujoco_commands", 10);

  return true;
}

void UR5eHardwareInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Update the latest joint state data
  latest_position_ = msg->position;
  latest_velocity_ = msg->velocity;
  latest_effort_ = msg->effort;
}

bool UR5eHardwareInterface::read(
  const ros::Time time, const ros::Duration period)
{
  // Read robot states from hardware
  ROS_INFO_NAMED("UR5eHardwareInterface", "Reading...");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // Update hardware states using the latest JointState data
    if (i < latest_position_.size())
    {
      hw_position_states_[i] = latest_position_[i];
      hw_velocity_states_[i] = latest_velocity_[i];
      hw_effort_states_[i] = latest_effort_[i];
    }
  }

  return true;
}

bool UR5eHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  // Write commands to hardware
  sensor_msgs::JointState mujoco_commands_msg;
  
  // Set the commands to be sent to the hardware
  mujoco_commands_msg.position = hw_position_commands_;
  // mujoco_commands_msg.velocity = hw_velocity_commands_;
  
  // Publish the commands
  mujoco_commands_pub.publish(mujoco_commands_msg);

  return true;
}

}  // namespace ur5e_hardware_interface
