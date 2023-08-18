#include "controller_manager/controller_manager.h"
#include "ur5e_hardware_interface/ur5e_hardware_interface.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "ur5e_hardware_interface");

  // Create an asynchronous spinner to handle ROS callbacks in a separate thread
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create ROS node handles for the root and robot namespaces
  ros::NodeHandle root_nh;
  ros::NodeHandle robot_nh("~");

  // Create an instance of the UR5e hardware interface
  ur5e_hardware_interface::UR5eHardwareInterface ur5e_hardware_interface;

  // Create a controller manager and associate it with the hardware interface
  controller_manager::ControllerManager controller_manager(&ur5e_hardware_interface, root_nh);

  // Initialize variables for timing
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  // Initialize the UR5e hardware interface
  ur5e_hardware_interface.init(root_nh, robot_nh);

  // Set the loop rate for control and execution
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    // Receive the current state from the robot
    if (!ur5e_hardware_interface.read(timestamp, period)) {
      ROS_FATAL_NAMED("ur5e_hardware_interface",
                      "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get the current time and calculate the elapsed time since the last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
      stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers with the current time and period
    controller_manager.update(timestamp, period);

    // Send the new setpoint to the robot
    ur5e_hardware_interface.write(timestamp, period);

    // Pause for the specified loop rate
    loop_rate.sleep();
  }

  // Stop the asynchronous spinner
  spinner.stop();

  ROS_INFO_NAMED("ur5e_hardware_interface", "Shutting down.");

  return 0;
}
