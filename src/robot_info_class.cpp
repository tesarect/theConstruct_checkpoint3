#include "robot_info/robot_info_class.h"
// #include <iostream>
// #include <robot_info/robot_info_class.h>

RobotInfo::RobotInfo(ros::NodeHandle &nh, const std::string &description,
                     const std::string &serial_no) {
  // Store NodeHandle reference as pointer
  this->nh = &nh;

  // Set mandatory fields
  robot_description = description;
  serial_number = serial_no;

  // Initialize optional fields with default values
  ip_address = "0.0.0.0";     // Default IP
  firmware_version = "1.0.0"; // Default firmware version

  ROS_INFO("RobotInfo initialized for the robot: %s (SN: %s)",
           robot_description.c_str(), serial_number.c_str());
}

// Virtual destructor
RobotInfo::~RobotInfo() {
  ROS_INFO("RobotInfo object destroyed for robot: %s",
           robot_description.c_str());
}

// Initialize publisher for robot_info topic using the existing message
void RobotInfo::init_publisher() {
  robot_info_publisher =
      this->nh->advertise<robotinfo_msgs::RobotInfo10Fields>("robot_info", 10);
  ROS_INFO("RobotInfo Publisher initialized");
}

// Getter methods
std::string RobotInfo::getRobotDescription() const { return robot_description; }

std::string RobotInfo::getSerialNumber() const { return serial_number; }

std::string RobotInfo::getIPAddress() const { return ip_address; }

std::string RobotInfo::getFirmwareVersion() const { return firmware_version; }

// Setter methods
void RobotInfo::setIpAddress(const std::string &ip_addr) {
  ip_address = ip_addr;
  ROS_INFO("IP Address updated to: %s", ip_addr.c_str());
}

void RobotInfo::setFirmwareVersion(const std::string &frmwr_ver) {
  firmware_version = frmwr_ver;
  ROS_INFO("Firmware version updated to: %s", frmwr_ver.c_str());
}

void RobotInfo::displayInfo() const {
  std::cout << "=== Robot Information ===" << std::endl;
  std::cout << "Description     : " << robot_description << std::endl;
  std::cout << "Serial Number   : " << serial_number << std::endl;
  std::cout << "IP Address      : " << ip_address << std::endl;
  std::cout << "Firmware Version: " << firmware_version << std::endl;
  std::cout << "=========================" << std::endl;
}

void RobotInfo::publish_data() {
  robotinfo_msgs::RobotInfo10Fields msg;

  msg.data_field_01 = "robot_description: " + robot_description;
  msg.data_field_02 = "serial_number: " + serial_number;
  msg.data_field_03 = "ip_address: " + ip_address;
  msg.data_field_04 = "firmware_version: " + firmware_version;
  msg.data_field_05 = "";
  msg.data_field_06 = "";
  msg.data_field_07 = "";
  msg.data_field_08 = "";
  msg.data_field_09 = "";
  msg.data_field_10 = "";

  // Publish the message
  robot_info_publisher.publish(msg);

  //   ROS_INFO("Published robot info: %s (SN: %s) - IP: %s, FW: %s",
  //            robot_description.c_str(), serial_number.c_str(),
  //            ip_address.c_str(), firmware_version.c_str());
}