#pragma once
#include "robot_info/robot_info_class.h"
#include "robot_info/hydraulic_system_monitor.h"
#include <string>

class AGVRobotInfo : public RobotInfo {
public:
  AGVRobotInfo(ros::NodeHandle &nh, const std::string &description,
               const std::string &serial_no);

  virtual ~AGVRobotInfo();

  void publish_data();
  void displayInfo() const;
  std::string getMaximumPayload() const;
  void setMaximumPayload(const std::string &payload);

  HydraulicSystemMonitor hydraulic_system;

  protected:
  //   void setMaximumPayload(const std::string &payload);

private:
  std::string maximum_payload;
};