#include "robot_info/robot_info_class.h"
// #include <robot_info/robot_info_class.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "robot_info_node");
  ros::NodeHandle nh;

  ROS_INFO("Robot Info Node Started");

  // Create RobotInfo object with mandatory parameters
  RobotInfo robot1_info(nh, "BaseClass Robot Arm Model X1", "SN123456789");

  // Initialize the publisher
  robot1_info.init_publisher();

  // Display initial robot information
  ROS_INFO("Initial robot information:");
  robot1_info.displayInfo();

  // Set some initial values for IP and firmware
  robot1_info.updateIpAddress("192.168.1.100");
  robot1_info.updateFirmwareVersion("v2.1.3");

  ROS_INFO("Updated robot information:");
  robot1_info.displayInfo();

//   // Test the getter methods
//   ROS_INFO("Testing getter methods:");
//   ROS_INFO("Description: %s", robot1_info.getRobotDescription().c_str());
//   ROS_INFO("Serial Number: %s", robot1_info.getSerialNumber().c_str());
//   ROS_INFO("IP Address: %s", robot1_info.getIPAddress().c_str());
//   ROS_INFO("Firmware Version: %s", robot1_info.getFirmwareVersion().c_str());

  // Set up publishing rate
  ros::Rate loop_rate(1); // 1 Hz

//   ROS_INFO("Starting to publish robot info on 'robot1_info' topic...");
//   ROS_INFO("You can listen to the topic using: rostopic echo /robot1_info");

  // Main loop - publish robot data periodically
  while (ros::ok()) {
    // Publish robot data
    robot1_info.publish_data();

    // Process ROS callbacks
    ros::spinOnce();

    // Sleep to maintain loop rate
    loop_rate.sleep();
  }

  ROS_INFO("Robot Info Node Shutting Down");
  return 0;
}