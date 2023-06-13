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

std::string GNSS_TOPIC = "/sensor_pose/gnss";
std::string BEARING_TOPIC = "/sensor_pose/bearing";
std::string ELVATION_TOPIC = "/sensor_pose/elevation";
std::string GYRO_TOPIC = "/sensor_pose/gyro";

double decdeg_convert(double degree_min)
{
    int dd = degree_min / 100;
    double mm = std::fmod(degree_min, 100);
    return dd + mm / 60;
}

int main(int argc, char **argv){

    // Check for correct arguments
    if (argc != 4)
    {
        std::cout << "(Cmd Line) Usage: bag_gps_merge <bag_path> <gnss_path> <output_path>" << std::endl << std::endl;
        std::cout << "(rosrun)   Usage: rosrun geolocation bag_gps_merge bag:=<BAG PATH> gps:=<GPS PATH> output:=<OUTPUT PATH>" << std::endl;
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

    std::ifstream gps_log_file(gnss_path);

    int line_count = 0;

    double tilt_x = 0;
    double tilt_y = 0;
    double tilt_z = 0;

    double gyro_time;
    int gyro_calib_counter = 0;
    int gyro_calib_max = 30;
    int gyro_calib_sum_x = 0;
    int gyro_calib_sum_y = 0;
    int gyro_calib_sum_z = 0;
    int gyro_offset_x = 0;
    int gyro_offset_y = 0;
    int gyro_offset_z = 0;
    bool gyro_calib = false;

    struct tm t = {0};

    if (!gps_log_file)
    {
        std::cout << "Error opening file" << std::endl;
        return 0;
    }

    // actual bag writing
    std::string line;
    while(std::getline(gps_log_file, line))
    {
        //std::cout << "line: " << line_count << std::endl << line << std::endl;
        // line checking and bag writing code here;

        // check start of line
        std::vector<std::string> params;
        std::string param;
        std::stringstream ss(line);
        while(getline(ss, param, ','))
        {
            params.push_back(param);
        }
        line_count += 1;
        if (params[0] == "$GPRMC")
        {
            sensor_msgs::NavSatFix gnss;

            // check for warnings
            if (params[2] == "V")
            {
                continue;
            }
            
            // populate timestamp struct
            t.tm_year = std::stoi(params[9].substr(4,2)) + 100;
            t.tm_mon = std::stoi(params[9].substr(2,2)) - 1;
            t.tm_mday = std::stoi(params[9].substr(0,2));
            t.tm_hour = std::stoi(params[1].substr(0,2));
            t.tm_min = std::stoi(params[1].substr(2,2));
            t.tm_sec = std::stoi(params[1].substr(4,2));

            time_t timestamp = mktime(&t) - timezone;

            // check if timestamp is within bag time
            if (ros::Time(timestamp) < start_time || ros::Time(timestamp) > end_time)
            {
                continue;
            }

            // populate message
            gnss.header.stamp = ros::Time(timestamp);
            gnss.header.frame_id = "base_link";
            if (params[4] == "N")
            {
                gnss.latitude = decdeg_convert(std::stod(params[3]));
            } else {
                gnss.latitude = -decdeg_convert(std::stod(params[3]));
            }
            if (params[6] == "E")
            {
                gnss.longitude = decdeg_convert(std::stod(params[5]));
            } else {
                gnss.longitude = decdeg_convert(-std::stod(params[5]));
            }

            // write to bag
            bag.write(GNSS_TOPIC, ros::Time(timestamp), gnss);
        }
        else if (params[0] == "$GPGGA")
        {
            sensor_msgs::NavSatFix gnss;

            // check for warnings
            if (params[6] == "0")
            {
                continue;
            }

            // populate timestamp struct
            t.tm_hour = std::stoi(params[1].substr(0,2));
            t.tm_min = std::stoi(params[1].substr(2,2));
            t.tm_sec = std::stoi(params[1].substr(4,2));

            time_t timestamp = mktime(&t) - timezone;

            // check if timestamp is within bag time
            if (ros::Time(timestamp) < start_time || ros::Time(timestamp) > end_time)
            {
                continue;
            }

            // populate message
            gnss.header.stamp = ros::Time(timestamp);
            gnss.header.frame_id = "base_link";
            if (params[3] == "N")
            {
                gnss.latitude = decdeg_convert(std::stod(params[2]));
            } else {
                gnss.latitude = -decdeg_convert(std::stod(params[2]));
            }
            if (params[5] == "E")
            {
                gnss.longitude = decdeg_convert(std::stod(params[4]));
            } else {
                gnss.longitude = -decdeg_convert(std::stod(params[4]));
            }
            gnss.altitude = std::stod(params[9]);

            // write to bag
            bag.write(GNSS_TOPIC, ros::Time(timestamp), gnss);
        }
        else if (params[0] == "$PAAG" && params[1] == "DATA")
        {
            /*
            if (params[2] == "G") // Gyroscope
            {
                double sensor_x = std::stod(params[4]);
                double sensor_y = std::stod(params[5]);
                double sensor_z = std::stod(params[6]);
                double gyro_x = (sensor_x - gyro_offset_x)/14.375;
                double gyro_y = (sensor_y - gyro_offset_y)/14.375;
                double gyro_z = (sensor_z - gyro_offset_z)/14.375;
                
                // init gyro time
                if (!gyro_time)
                {
                    gyro_time = std::stod(params[3]);
                    continue;
                }

                // "calibrate" gyro offsets
                if (!gyro_calib)
                {
                    gyro_calib_sum_x += gyro_x;
                    gyro_calib_sum_y += gyro_y;
                    gyro_calib_sum_z += gyro_z;
                    gyro_calib_counter++;
                    if (gyro_calib_counter == gyro_calib_max)
                    {
                        gyro_calib = true;
                        gyro_offset_x = gyro_calib_sum_x / gyro_calib_max;
                        gyro_offset_y = gyro_calib_sum_y / gyro_calib_max;
                        gyro_offset_z = gyro_calib_sum_z / gyro_calib_max;
                    }
                    else
                    {
                        continue;
                    }
                }

                tilt_x += gyro_x * (std::stod(params[3]) - gyro_time);
                tilt_y += gyro_y * (std::stod(params[3]) - gyro_time);
                tilt_z += gyro_z * (std::stod(params[3]) - gyro_time);
                gyro_time = std::stod(params[3]);

                geometry_msgs::Vector3 gyro;
                gyro.x = tilt_x;
                gyro.y = tilt_y;
                gyro.z = tilt_z;
                bag.write(GYRO_TOPIC, ros::Time::now(), gyro);
            }*/
            if (params[2] == "C") // Compass
            {
                double sensor_x = std::stod(params[3]);
                double sensor_y = std::stod(params[4]);
                double sensor_z = std::stod(params[5]);
                double gauss_x = sensor_x/1090;
                double gauss_y = sensor_y/1090;
                double gauss_z = sensor_z/1090;
                double bearing_rad = atan2(gauss_y, gauss_x);
                double bearing_deg = bearing_rad * 180 / M_PI;
                double elevation_rad = atan2(gauss_z, sqrt(gauss_x*gauss_x + gauss_y*gauss_y));
                double elevation_deg = elevation_rad * 180 / M_PI;
                std_msgs::Float32 bearing;
                bearing.data = bearing_deg;
                std_msgs::Float32 elevation;
                elevation.data = elevation_deg;

                t.tm_hour = std::stoi(params[3].substr(0,2));
                t.tm_min = std::stoi(params[3].substr(2,2));
                t.tm_sec = std::stoi(params[3].substr(4,2));

                time_t timestamp = mktime(&t) - timezone;

                if (!t.tm_year)
                {
                    continue;
                }

                // check if timestamp is within bag time
                if (ros::Time(timestamp) < start_time || ros::Time(timestamp) > end_time)
                {
                    continue;
                }

                bag.write(BEARING_TOPIC, ros::Time(timestamp), bearing);
                bag.write(ELVATION_TOPIC, ros::Time(timestamp), elevation);
            }
        }
    }
    gps_log_file.close();
    bag.close();
}
