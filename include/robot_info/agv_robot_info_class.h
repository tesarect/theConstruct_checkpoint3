#pragma once
#include "robot_info/robot_info_class.h"
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

  // protected:
  //   void setMaximumPayload(const std::string &payload);

private:
  std::string maximum_payload;
};