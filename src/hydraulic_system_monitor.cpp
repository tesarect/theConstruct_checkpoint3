#include "robot_info/hydraulic_system_monitor.h"
#include <string>

HydraulicSystemMonitor::HydraulicSystemMonitor(const std::string &oil_temp,
                                               const std::string &tank_level,
                                               const std::string &oil_pressure)
    : hydraulic_oil_temperature(oil_temp),
      hydraulic_oil_tank_fill_level(oil_temp),
      hydraulic_oil_pressure(oil_pressure) {
  ROS_INFO("HydraulicSystemMonitor initialized with default values");
}

HydraulicSystemMonitor::~HydraulicSystemMonitor() {
  ROS_INFO("HydraulicSystemMonitor destroyed");
}

std::string HydraulicSystemMonitor::getHydraulicOilTemperature() const {
  return hydraulic_oil_temperature;
}

std::string HydraulicSystemMonitor::getHydraulicOilTankFillLevel() const {
  return hydraulic_oil_tank_fill_level;
}

std::string HydraulicSystemMonitor::getHydraulicOilPressure() const {
  return hydraulic_oil_pressure;
}

void HydraulicSystemMonitor::setHydraulicOilTemperature(
    const std::string &temp) {
  hydraulic_oil_temperature = temp;
  ROS_INFO("Hydraulic oil temperature updated to: %sC", temp.c_str());
}

void HydraulicSystemMonitor::setHydraulicOilTankFillLevel(
    const std::string &tnk_level) {
  hydraulic_oil_tank_fill_level = tnk_level;
  ROS_INFO("Hydraulic oil tank fill level updated to: %s%%", tnk_level.c_str());
}

void HydraulicSystemMonitor::setHydraulicOilPressure(
    const std::string &pressure) {
  hydraulic_oil_pressure = pressure;
  ROS_INFO("Hydraulic oil pressure updated to: %s bar", pressure.c_str());
}

std::string HydraulicSystemMonitor::getHydraulicInfo() const {
  return "Hydraulic Oil - temp : %sC | tank fill level : %s%% | pressure : %s "
         "bar",
         hydraulic_oil_temperature.c_str(),
         hydraulic_oil_tank_fill_level.c_str(), hydraulic_oil_pressure.c_str();
}

void HydraulicSystemMonitor::update(const std::string temp,
                                    const std::string tnk_level,
                                    std::string pressure) {
  hydraulic_oil_temperature = temp;
  hydraulic_oil_tank_fill_level = tnk_level;
  hydraulic_oil_pressure = pressure;
  ROS_INFO("Hydraulic system updated: %s, %s, %s", temp.c_str(),
           tnk_level.c_str(), pressure.c_str());
}

// void HydraulicSystemMonitor::displaySystemStatus() const {
//   std::cout << "=== Hydraulic System Status ===" << std::endl;
//   std::cout << "Oil Temperature    : " << hydraulic_oil_temperature
//             << std::endl;
//   std::cout << "Tank Fill Level    : " << hydraulic_oil_tank_fill_level
//             << std::endl;
//   std::cout << "Oil Pressure       : " << hydraulic_oil_pressure <<
//   std::endl; std::cout << "===============================" << std::endl;
// }