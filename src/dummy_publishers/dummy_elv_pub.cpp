/**
 * @file dummy_elv_pub.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Publishes an elevation parameter value to the elevation topic
 * @version 1.1.0
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"

/**
 * @brief Main function that publishes an elevation parameter value to the elevation topic
 * 
 * @return int Returns 0 if successful
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_elevation_pub");
  ros::NodeHandle n;

  std::string ELEVATION_TOPIC;
  ros::param::get("/elevation_topic", ELEVATION_TOPIC);

  ros::Publisher pub = n.advertise<std_msgs::Float32>(ELEVATION_TOPIC, 10);

  int ros_rate = 30;
  ros::param::get("/rate", ros_rate);
  ros::Rate rate(ros_rate);
  double elevation;
  
  while (ros::ok())
  {
    std_msgs::Float32 elv;
    ros::param::get("/dummy_elevation", elevation);
    elv.data = elevation;
    pub.publish(elv);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
