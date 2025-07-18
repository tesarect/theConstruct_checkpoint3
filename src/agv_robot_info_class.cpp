#include "robot_info/agv_robot_info_class.h"
#include "robot_info/hydraulic_system_monitor.h"
#include "ros/node_handle.h"
#include <iostream>

AGVRobotInfo::AGVRobotInfo(ros::NodeHandle &nh, const std::string &description,
                           const std::string &serial_no)
    : RobotInfo(nh, description, serial_no),
      hydraulic_system("40", "50", "200") {

  // Default values
  setIpAddress("192.168.1.1");
  setFirmwareVersion("v1.0");
  maximum_payload = "100";

  ROS_INFO("AGVRobotInfo initialized for AGV: %s", description.c_str());

  //   // Hydraulic system monitor initialization
  //   HydraulicSystemMonitor hydraulic_system("40", "50", "200");
}

AGVRobotInfo::~AGVRobotInfo() {
  ROS_INFO("RobotInfo object destroyed for robot: %s",
           getRobotDescription().c_str());
}

std::string AGVRobotInfo::getMaximumPayload() const { return maximum_payload; }

void AGVRobotInfo::setMaximumPayload(const std::string &payload) {
  maximum_payload = payload;
  ROS_INFO("Maximum payload updated to: %s Kg", maximum_payload.c_str());
}

void AGVRobotInfo::displayInfo() const {
  std::cout << "=== AGV Robot Information ===" << std::endl;
  std::cout << "Description     : " << getRobotDescription() << std::endl;
  std::cout << "Serial Number   : " << getSerialNumber() << std::endl;
  std::cout << "IP Address      : " << getIPAddress() << std::endl;
  std::cout << "Firmware Version: " << getFirmwareVersion() << std::endl;
  std::cout << "Maximum Payload : " << getMaximumPayload() << " Kg"
            << std::endl;
  std::cout << "Oil Temperature    : "
            << hydraulic_system.getHydraulicOilTemperature() << std::endl;
  std::cout << "Tank Fill Level    : "
            << hydraulic_system.getHydraulicOilTankFillLevel() << std::endl;
  std::cout << "Oil Pressure       : "
            << hydraulic_system.getHydraulicOilPressure() << std::endl;
  std::cout << "=========================" << std::endl;
}

void AGVRobotInfo::publish_data() {
  robotinfo_msgs::RobotInfo10Fields msg;

  msg.data_field_01 = "robot_description: " + getRobotDescription();
  msg.data_field_02 = "serial_number: " + getSerialNumber();
  msg.data_field_03 = "ip_address: " + getIPAddress();
  msg.data_field_04 = "firmware_version: " + getFirmwareVersion();
  msg.data_field_05 = "maximum_payload: " + getMaximumPayload() + " Kg";
  msg.data_field_06 = "hydraulic_oil_temperature: " +
                      hydraulic_system.getHydraulicOilTemperature() + "C";
  msg.data_field_07 = "hydraulic_oil_tank_fill_level: " +
                      hydraulic_system.getHydraulicOilTankFillLevel() + "%";
  msg.data_field_08 =
      "hydraulic_oil_pressure: " + hydraulic_system.getHydraulicOilPressure() +
      " bar";
  msg.data_field_09 = "";
  msg.data_field_10 = "";

  // Publish the message
  robot_info_publisher.publish(msg);

  //   ROS_INFO("Published robot info: %s (SN: %s) - IP: %s, FW: %s, MaxPayload:
  //   %s",
  //            getRobotDescription().c_str(), getSerialNumber().c_str(),
  //            getIPAddress().c_str(), getFirmwareVersion().c_str(),
  //            getMaximumPayload().c_str());
}