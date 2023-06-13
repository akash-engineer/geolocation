#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <time.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/NavSatFix.h"
#include "csv.hpp"

std::string GNSS_TOPIC = "/sensor_pose/gnss";
std::string BEARING_TOPIC = "/sensor_pose/bearing";
std::string ELVATION_TOPIC = "/sensor_pose/elevation";
std::string ROLL_TOPIC = "/sensor_pose/roll";

int main(int argc, char **argv){

    // Check for correct arguments
    if (argc != 4)
    {
        std::cout << "(Cmd Line) Usage: bag_csv_merge <bag_path> <gps_path> <output_path>" << std::endl << std::endl;
        std::cout << "(rosrun)   Usage: rosrun geolocation bag_csv_merge bag:=<BAG PATH> GPS:=<GPS PATH> output:=<OUTPUT PATH>" << std::endl;
        return 0;
    }
    
    std::string bag_path = argv[1];
    std::string gnss_path = argv[2];
    std::string output_path = argv[3];
    
    std::filesystem::copy(bag_path, output_path);

    rosbag::Bag source_bag;

    // get start and end times of bag

    source_bag.open(output_path, rosbag::bagmode::Read);
    rosbag::View view(source_bag);
    ros::Time start_time = view.getBeginTime();
    ros::Time end_time = view.getEndTime();
    std::cout << std::endl << "Start time: " << start_time << std::endl;
    std::cout << "End time: " << end_time << std::endl << std::endl;
    source_bag.close();

    rosbag::Bag bag(output_path, rosbag::bagmode::Append);
    csv::CSVReader gps_csv_file(gnss_path);

    int rows = 0;

    for (csv::CSVRow& row : gps_csv_file)
    {
        auto const local_date = row["date"].get<std::string>();
        auto const local_time = row["time(local time)"].get<std::string>();
        auto const altitude   = row["altitude (m)"].get<double>();
        auto const longitude  = row["longitude"].get<double>();
        auto const latitude   = row["latitude"].get<double>();
        auto const tilt       = (row["tilt (deg)"].get<double>())/180*M_PI;
        auto const roll       = (row["roll (deg)"].get<double>())/180*M_PI;
        auto const bearing    = (row["compass (deg)"].get<double>())/180*M_PI;

        struct tm t = {0};
        std::string date_time = local_date + " " + local_time;
        strptime(date_time.c_str(), "%Y-%m-%d %H:%M:%S", &t);
        time_t epoch_time = mktime(&t);

        sensor_msgs::NavSatFix gnss;
        gnss.header.stamp = ros::Time(epoch_time);
        gnss.header.frame_id = "base_link";
        gnss.latitude = latitude;
        gnss.longitude = longitude;
        gnss.altitude = altitude;
        bag.write(GNSS_TOPIC, ros::Time(epoch_time), gnss);

        std_msgs::Float32 bearing_msg;
        bearing_msg.data = bearing;
        bag.write(BEARING_TOPIC, ros::Time(epoch_time), bearing_msg);

        std_msgs::Float32 elevation_msg;
        elevation_msg.data = tilt;
        bag.write(ELVATION_TOPIC, ros::Time(epoch_time), elevation_msg);

        std_msgs::Float32 roll_msg;
        roll_msg.data = roll;
        bag.write(ROLL_TOPIC, ros::Time(epoch_time), roll_msg);

        rows += 1;
    }

    bag.close();
    std::cout << "Bag merged successfully" << std::endl;
    std::cout << "Rows added: " << rows << std::endl;
}