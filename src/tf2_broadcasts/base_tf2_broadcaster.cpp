/**
 * @file base_tf2_broadcaster.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Broadcasts the transform from the base_link to innovusion (and any other child frames)
 * @version 0.1.0
 * @date 2023-05-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

/**
 * @brief Main function that broadcasts the transform from the base_link to innovusion (and any other child frames)
 * 
 * @return int Returns 0 if successful
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "base_tf2_broadcaster");

    ros::NodeHandle n;

    // Read and set the rate of the tf2 broadcaster. Default is 30/s.
    int r = 30;
    ros::param::get("/rate", r);
    ros::Rate rate(r);

    static tf2_ros::TransformBroadcaster br;

    /*
    Read and set the transform from base_link to innovusion. Default is 0,0,0.
    This is to correct for the different axes of the Innovusion sensor.
    Angles are in radians.
    */
    double innovusion_tf_r = 0;
    double innovusion_tf_p = 0;
    double innovusion_tf_y = 0;
    ros::param::get("/innovusion_tf_r", innovusion_tf_r);
    ros::param::get("/innovusion_tf_p", innovusion_tf_p);
    ros::param::get("/innovusion_tf_y", innovusion_tf_y);

    while (ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "innovusion";

        /*
        Position of Innovusion relative to base_link.
        Should be left at 0,0,0.
        */
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;

        /*
        Orientation of Innovusion relative to base_link.
        */
        tf2::Quaternion q;
        q.setRPY(innovusion_tf_r, innovusion_tf_p, innovusion_tf_y);
        q.normalize();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};