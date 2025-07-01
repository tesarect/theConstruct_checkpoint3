#pragma once
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <string>

class RobotInfo {
public:
  RobotInfo(ros::NodeHandle &nh, const std::string &description,
            const std::string &serial_no);

  virtual ~RobotInfo();

  virtual void publish_data();
  void displayInfo() const;
  std::string getRobotDescription() const;
  std::string getSerialNumber() const;
  std::string getIPAddress() const;
  std::string getFirmwareVersion() const;

  void init_publisher();
  void updateIpAddress(const std::string &ip_addr) { setIpAddress(ip_addr); }
  void updateFirmwareVersion(const std::string &frmwr_ver) { setFirmwareVersion(frmwr_ver); }


protected:
  ros::NodeHandle *nh;
  ros::Publisher robot_info_publisher;
  std::string robot_description;
  std::string serial_number;
  std::string ip_address;
  std::string firmware_version;

//   void init_publisher();
  void setIpAddress(const std::string &ip_addr);
  void setFirmwareVersion(const std::string &frmwr_ver);

private:
};