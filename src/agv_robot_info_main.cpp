#include "robot_info/agv_robot_info_class.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "agv_robot_info_node");
  ros::NodeHandle nh;

  ROS_INFO("AGV Robot Info Node Started");

  // Create RobotInfo object with mandatory parameters
  AGVRobotInfo agv_robot_info(nh, "Mir100", "567A359");

  // Initialize the publisher
  agv_robot_info.init_publisher();

  // Display initial robot information
  ROS_INFO("Initial robot information:");
//   agv_robot_info.displayInfo();

  // Set some initial values for IP and firmware
  agv_robot_info.updateIpAddress("169.254.5.180");
  agv_robot_info.updateFirmwareVersion("3.5.8");
  //   agv_robot_info.setMaximumPayload(std::to_string(68));

  ROS_INFO("Updated robot information:");
  agv_robot_info.displayInfo();

  // Test the getter methods
//   ROS_INFO("Testing getter methods:");
//   ROS_INFO("Description     : %s",
//            agv_robot_info.getRobotDescription().c_str());
//   ROS_INFO("Serial Number   : %s", agv_robot_info.getSerialNumber().c_str());
//   ROS_INFO("IP Address      : %s", agv_robot_info.getIPAddress().c_str());
//   ROS_INFO("Firmware Version: %s", agv_robot_info.getFirmwareVersion().c_str());
//   ROS_INFO("Maximum Payload : %s Kg",
//            agv_robot_info.getMaximumPayload().c_str());

  // Set up publishing rate
  ros::Rate loop_rate(1); // 1 Hz

  ROS_INFO("Starting to publish robot info on 'robot_info' topic...");
  ROS_INFO("You can listen to the topic using: rostopic echo /robot_info");

  // Main loop - publish robot data periodically
  while (ros::ok()) {
    // Publish robot data
    agv_robot_info.publish_data();

    // Process ROS callbacks
    ros::spinOnce();

    // Sleep to maintain loop rate
    loop_rate.sleep();
  }

  ROS_INFO("Robot Info Node Shutting Down");
  return 0;
}