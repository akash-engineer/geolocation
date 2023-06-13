/**
 * @file dummy_markers_pub.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Publishes a series of moving points to the marker array topic.
 * @version 1.1.0
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_pt_pub");
  ros::NodeHandle n;

  std::string MARKER_ARRAY_TOPIC;
  ros::param::get("/marker_array_topic", MARKER_ARRAY_TOPIC);

  ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC, 10);

  int ros_rate = 30;
  ros::param::get("/rate", ros_rate);
  int SPEED_MULT = 5;
  ros::Rate rate(ros_rate);
  float count = 0;
  float max_count = 50*ros_rate;
  int direction_right = 1;
  while (ros::ok())
  {
    geometry_msgs::Point pt0, pt1, pt2;
    pt0.x = count/ros_rate;
    pt0.y = 50;
    pt0.z = 3.0;
    pt1.x = 10;
    pt1.y = -count/ros_rate;
    pt1.z = 6.0;
    pt2.x = count/ros_rate;
    pt2.y = 20;
    pt2.z = abs(count/ros_rate);

    count+=(SPEED_MULT*direction_right);
    if (count > max_count || count < -max_count)
    {
      direction_right = -direction_right;
    }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker0, marker1, marker2;

    marker0.header.frame_id = "base_link";
    marker0.header.stamp = ros::Time();
    marker0.ns = "dummy_markers";
    marker0.id = 0;
    marker0.type = visualization_msgs::Marker::SPHERE;
    marker0.action = visualization_msgs::Marker::ADD;
    marker0.pose.position = pt0;
    marker0.pose.orientation.x = 0.0;
    marker0.pose.orientation.y = 0.0;
    marker0.pose.orientation.z = 0.0;
    marker0.pose.orientation.w = 1.0;
    marker0.scale.x = 5;
    marker0.scale.y = 5;
    marker0.scale.z = 5;
    marker0.color.a = 1;
    marker0.color.r = 1;

    marker1.header.frame_id = "base_link";
    marker1.header.stamp = ros::Time();
    marker1.ns = "dummy_markers";
    marker1.id = 1;
    marker1.type = visualization_msgs::Marker::SPHERE;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.position = pt1;
    marker1.pose.orientation.x = 0.0;
    marker1.pose.orientation.y = 0.0;
    marker1.pose.orientation.z = 0.0;
    marker1.pose.orientation.w = 1.0;
    marker1.scale.x = 5;
    marker1.scale.y = 5;
    marker1.scale.z = 5;
    marker1.color.a = 1.0;
    marker1.color.g = 1;

    marker2.header.frame_id = "base_link";
    marker2.header.stamp = ros::Time();
    marker2.ns = "dummy_markers";
    marker2.id = 2;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position = pt2;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 5;
    marker2.scale.y = 5;
    marker2.scale.z = 5;
    marker2.color.a = 1;
    marker2.color.b = 1;

    marker_array.markers.push_back(marker0);
    marker_array.markers.push_back(marker1);
    marker_array.markers.push_back(marker2);

    pub.publish(marker_array);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
