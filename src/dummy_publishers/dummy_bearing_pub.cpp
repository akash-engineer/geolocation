/**
 * @file dummy_bearing_pub.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Publishes a bearing parameter value to the bearing topic
 * @version 1.1.0
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"

/**
 * @brief Main function that publishes a bearing parameter value to the bearing topic
 * 
 * @return int Returns 0 if successful
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_bearing_pub");
  ros::NodeHandle n;

  std::string BEARING_TOPIC;
  ros::param::get("/bearing_topic", BEARING_TOPIC);

  ros::Publisher pub = n.advertise<std_msgs::Float32>(BEARING_TOPIC, 10);

  int ros_rate = 30;
  ros::param::get("/rate", ros_rate);
  ros::Rate rate(ros_rate);
  double bearing;
  
  while (ros::ok())
  {
    std_msgs::Float32 brng;
    ros::param::get("/dummy_bearing", bearing);
    brng.data = bearing;
    pub.publish(brng);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
