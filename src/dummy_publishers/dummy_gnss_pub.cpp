/**
 * @file dummy_gnss_pub.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Publishes a GNSS coordinate parameter to the gnss topic
 * @version 1.1.0
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_gnss_pub");
  ros::NodeHandle n;

  std::string GNSS_TOPIC;
  ros::param::get("/gnss_topic", GNSS_TOPIC);

  ros::Publisher pub = n.advertise<sensor_msgs::NavSatFix>(GNSS_TOPIC, 10);

  int ros_rate = 30;
  ros::param::get("/rate", ros_rate);
  ros::Rate rate(ros_rate);
  while (ros::ok())
  {
    double lon, lat, alt;
    ros::param::get("/dummy_longitude", lon);
    ros::param::get("/dummy_latitude", lat);
    ros::param::get("/dummy_altitude", alt);
    sensor_msgs::NavSatFix gnss;
    gnss.header.stamp = ros::Time::now();
    gnss.header.frame_id = "map";
    gnss.longitude = lon;
    gnss.latitude = lat;
    gnss.altitude = alt;

    pub.publish(gnss);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
