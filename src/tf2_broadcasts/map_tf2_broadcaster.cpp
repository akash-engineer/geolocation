/**
 * @file map_tf2_broadcaster.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief Broadcasts the transform from map to base_link
 * @version 1.0.0
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <math.h>

 /*! Bearing of the sensor in degrees */
float bearing = 0;
/*! Elevation of the sensor in degrees */
float elevation = 0;
/*! Roll of the sensor in degrees */
float roll = 0;
/*! Altitude of the sensor in meters */
float altitude = 0;

/**
 * @brief Callback function for the elevation topic
 * 
 * @param sensor_elv The elevation value
 */
void elvCallback(const std_msgs::Float32ConstPtr &sensor_elv)
{
    elevation = sensor_elv->data;    
}

/**
 * @brief Callback function for the bearing topic
 * 
 * @param sensor_brng The bearing value
 */
void brngCallback(const std_msgs::Float32ConstPtr &sensor_brng)
{
    bearing = sensor_brng->data;
}

/**
 * @brief Callback function for the roll topic
 * 
 * @param sensor_roll The roll value
 */
void rollCallback(const std_msgs::Float32ConstPtr &sensor_roll)
{
    roll = sensor_roll->data;
}

/**
 * @brief Callback function for the gnss topic
 * 
 * @param sensor_gnss The gnss coordinate
 */
void gnssCallback(const sensor_msgs::NavSatFixConstPtr &sensor_gnss)
{
    altitude = sensor_gnss->altitude;
}

/**
 * @brief Main function that broadcasts the transform from map to base_link
 * 
 * @return int Returns 0 if successful
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "map_tf2_broadcaster");

    ros::NodeHandle n;

    std::string ELV_TOPIC;
    std::string BRNG_TOPIC;
    std::string ROLL_TOPIC;
    std::string GNSS_TOPIC;
    ros::param::get("/elevation_topic", ELV_TOPIC);
    ros::param::get("/bearing_topic", BRNG_TOPIC);
    ros::param::get("/roll_topic", ROLL_TOPIC);
    ros::param::get("/gnss_topic", GNSS_TOPIC);

    ros::Subscriber elv = n.subscribe(ELV_TOPIC, 10, &elvCallback);
    ros::Subscriber brng = n.subscribe(BRNG_TOPIC, 10, &brngCallback);
    ros::Subscriber rol = n.subscribe(ROLL_TOPIC, 10, &rollCallback);
    ros::Subscriber gnss = n.subscribe(GNSS_TOPIC, 10, &gnssCallback);

    int r = 30;
    ros::param::get("/rate", r);
    ros::Rate rate(r);

    double ground_height = 0;
    
    static tf2_ros::TransformBroadcaster br;

    while (ros::ok())
    {
        ros::param::get("/ground_height", ground_height);
        
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = altitude - ground_height;

        tf2::Quaternion q;
        q.setRPY(elevation, roll, bearing);
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