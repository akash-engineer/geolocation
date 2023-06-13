/**
 * @file dummy_pt_pub.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Publishes a moving point to the point topic.
 * @version 1.1.0
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_pt_pub");
  ros::NodeHandle n;

  std::string POINT_TOPIC;
  ros::param::get("/point_topic", POINT_TOPIC);

  ros::Publisher pub = n.advertise<geometry_msgs::Point>(POINT_TOPIC, 10);

  int ros_rate = 30;
  ros::param::get("/rate", ros_rate);
  int SPEED_MULT = 5;
  ros::Rate rate(ros_rate);
  float count = 0;
  float max_count = 50*ros_rate;
  int direction_right = 1;
  while (ros::ok())
  {
    geometry_msgs::Point pt;
    pt.x = count/ros_rate;
    pt.y = 50;
    pt.z = 3.0;
    pub.publish(pt);

    count+=(SPEED_MULT*direction_right);
    if (count > max_count || count < -max_count)
    {
      direction_right = -direction_right;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
