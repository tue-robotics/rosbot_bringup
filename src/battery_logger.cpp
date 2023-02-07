#include "ros/ros.h"
#include <sensor_msgs/BatteryState.h>

#define WARNING_VOLTAGE 10.5
#define ERROR_VOLTAGE 10

#include <ros/console.h>

class BatteryLogger
{
public:
  BatteryLogger()
  {
    ros::NodeHandle nh;
    battery_sub = nh.subscribe<sensor_msgs::BatteryState>("battery", 1, &BatteryLogger::batterycallback, this);
  }

private:
  void batterycallback(const sensor_msgs::BatteryState::ConstPtr &msg)
  {
    double value = msg->voltage;
    if (value < ERROR_VOLTAGE)
    {
      ROS_ERROR("Current Battery Voltage %.2f. Consider Charging the ROSBOT", value);
    }
    else if (value < WARNING_VOLTAGE)
    {
      ROS_WARN("Current Battery Voltage %.2f. Consider Charging the ROSBOT", value);
    }
    else
    {
      ROS_INFO("Current Battery Voltage %.2f.", value);
    }
  };

  ros::NodeHandle n;
  ros::Subscriber battery_sub;
};

/**
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_logger");

  ROS_INFO("started logger node");
  BatteryLogger batteryLogger = BatteryLogger();

  ros::Rate r(1 / 60.0);

  ROS_INFO("spinning");

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
