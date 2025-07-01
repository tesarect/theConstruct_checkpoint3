#pragma once
#include <ros/ros.h>
#include <string>

class HydraulicSystemMonitor {
public:
  //   std::string hydraulic_oil_temperature;
  //   std::string hydraulic_oil_tank_fill_level;
  //   std::string hydraulic_oil_pressure;

  HydraulicSystemMonitor() = default;
  HydraulicSystemMonitor(const std::string &oil_temp,
                         const std::string &tank_level,
                         const std::string &oil_pressure);
  ~HydraulicSystemMonitor();
  std::string getHydraulicOilTemperature() const;
  std::string getHydraulicOilTankFillLevel() const;
  std::string getHydraulicOilPressure() const;
  void setHydraulicOilTemperature(const std::string &temp);
  void setHydraulicOilTankFillLevel(const std::string &tnk_level);
  void setHydraulicOilPressure(const std::string &pressure);
  std::string getHydraulicInfo() const;
  void update(const std::string temp, const std::string tnk_level,
              std::string pressure);
  //   void displaySystemStatus() const;

protected:
private:
  std::string hydraulic_oil_temperature;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_pressure;
};