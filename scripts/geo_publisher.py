"""@package docstring
File: geo_publisher.py
Author: Richard Loong (richardloongcj@gmail.com)
Description: Read sensor and point coordinates and pose from ROS topics
             and publish the GNSS coordinates of the point to a ROS topic
Version: 1.1.1
Date: 2023-06-08
"""

#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import NavSatFix

# Global Variables

MARKER_ARRAY = MarkerArray()
SENSOR_LAT = 0
SENSOR_LON = 0
SENSOR_ALTITUDE = 0
SENSOR_BEARING = 0
SENSOR_ELEVATION = 0

# Geolocation functions

def point_transform(point, bearing, elevation):
    """Transforms the point from the sensor frame to the map frame

    :param point: Array representing the point coordinates in the sensor frame
    :type point: numpy array
    :param bearing: Bearing of the sensor in degrees
    :type bearing: float
    :param elevation: Elevation of the sensor in degrees
    :type elevation: float

    :return: Array representing the point coordinates in the map frame
    :rtype: numpy array
    """
    point = np.append(point, 1) # Convert to homogenous coordinates

    theta_x = np.deg2rad(-elevation)
    theta_z = np.deg2rad(bearing)

    r_x = np.array([[1, 0, 0],
               [0, np.cos(theta_x), np.sin(theta_x)],
               [0, -np.sin(theta_x), np.cos(theta_x)]])

    r_z = np.array([[np.cos(theta_z), np.sin(theta_z), 0],
                [-np.sin(theta_z), np.cos(theta_z), 0],
                [0, 0, 1]])

    t_matrix = np.zeros((4,4))
    t_matrix[:3, :3] = r_z.dot(r_x)
    t_matrix[:3, 3] = np.array([0,0,0])
    t_matrix[3, 3] = 1

    point_transformed = t_matrix.dot(point)
    point_cartesian = point_transformed[:3] / point_transformed[3]

    # print(point_cartesian)

    return point_cartesian


def find_point_bearing(point):
    """Returns the bearing of the point in degrees

    :param point: Array representing the point coordinates in the map frame
    :type point: numpy array

    :return: Bearing of the point in degrees
    :rtype: float
    """
    return np.rad2deg(np.arctan(point[0]/point[1]))


def find_nadir_dist(point):
    """Returns the nadir distance of the point in meters

    :param point: Array representing the point coordinates in the map frame
    :type point: numpy array

    :return: Nadir distance of the point in meters
    :rtype: float
    """
    return np.sqrt(point[0]**2 + point[1]**2)


def find_point_gnss(start_lat, start_lon, nadir_dist, point_bearing):
    """Returns the GNSS coordinates of the point

    :param start_lat: Lattitude of the sensor in degrees
    :type start_lat: float
    :param start_lon: Longitude of the sensor in degrees
    :type start_lon: float
    :param nadir_dist: Nadir distance of the point from the sensor in meters
    :type nadir_dist: float
    :param point_bearing: Absolute bearing of the point from the sensor in degrees
    :type point_bearing: float

    :return: Lattitude and longitude of the point in degrees
    :rtype: float, float
    """
    earth_radius = 6378.1 #Radius of the Earth
    brng = np.deg2rad(point_bearing) #Bearing is 90 degrees converted to radians.
    nadir_km = nadir_dist/1000 #Distance in km

    lat1 = np.deg2rad(start_lat) #Current lat point converted
    lon1 = np.deg2rad(start_lon) #Current long point converted
    lat2 = np.arcsin(np.sin(lat1)*np.cos(nadir_km/earth_radius)
                     + np.cos(lat1)*np.sin(nadir_km/earth_radius)*np.cos(brng))
    lon2 = lon1 + np.arctan2(np.sin(brng)*np.sin(nadir_km/earth_radius)
                             *np.cos(lat1), np.cos(nadir_km/earth_radius)-np.sin(lat1)*np.sin(lat2))

    return np.rad2deg(lat2), np.rad2deg(lon2)


# ROS-related functions
def marker_array_callback(data):
    """Callback function for the /track/marker_array topic. Updates the global variable MARKER_ARRAY.

    :param data: MarkerArray message from the /track/marker_array topic
    :type data: MarkerArray
    """
    global MARKER_ARRAY
    MARKER_ARRAY = data


def gnss_callback(data):
    """Callback function for the /sensor_pose/gnss topic. Updates the global variables SENSOR_LAT, SENSOR_LON and SENSOR_ALTITUDE.

    :param data: NavSatFix message from the /sensor_pose/gnss topic
    :type data: NavSatFix
    """
    global SENSOR_LAT, SENSOR_LON, SENSOR_ALTITUDE
    SENSOR_LAT = data.latitude
    SENSOR_LON = data.longitude
    SENSOR_ALTITUDE = data.altitude


def bearing_callback(data):
    """Callback function for the /sensor_pose/bearing topic. Updates the global variable SENSOR_BEARING.

    :param data: Float32 message from the /sensor_pose/bearing topic
    :type data: Float32
    """
    global SENSOR_BEARING
    SENSOR_BEARING = data.data


def elevation_callback(data):
    """Callback function for the /sensor_pose/elevation topic. Updates the global variable SENSOR_ELEVATION.

    :param data: Float32 message from the /sensor_pose/elevation topic
    :type data: Float32
    """
    global SENSOR_ELEVATION
    SENSOR_ELEVATION = data.data


def multi_track_publisher():
    """Publishes the GNSS coordinates of multiple points to the /track/gnss topic

    Loops through each marker in MARKER_ARRAY and publishes the GNSS coordinates of the point to the /tracker<X>/gnss topic.
    Creates and destroys topics as the number of markers in MARKER_ARRAY changes.
    """
    ros_rate = 30
    rospy.get_param("/ros_rate", ros_rate)
    rate = rospy.Rate(ros_rate)
    while not rospy.is_shutdown():
        array_len = len(MARKER_ARRAY.markers)
        for i in range(array_len):
            try:
                tracker_topic = "tracker" + str(i) + "/gnss"
                pub = rospy.Publisher(tracker_topic, NavSatFix, queue_size=10)
                point = np.array([MARKER_ARRAY.markers[i].pose.position.x,
                                MARKER_ARRAY.markers[i].pose.position.y,
                                MARKER_ARRAY.markers[i].pose.position.z])
                point_tf = point_transform(point, SENSOR_BEARING, SENSOR_ELEVATION)
                point_bearing = find_point_bearing(point_tf)
                nadir_dist = find_nadir_dist(point_tf)

                track_lat, track_lon = find_point_gnss(SENSOR_LAT,
                                                    SENSOR_LON,
                                                    nadir_dist,
                                                    point_bearing)
                point_altitude = point_tf[2]

                track = NavSatFix()
                track.header.stamp = rospy.Time.now()
                track.header.frame_id = 'track_pose'
                track.latitude = track_lat
                track.longitude = track_lon
                track.altitude = point_altitude
                pub.publish(track)
            except IndexError:
                break

        rate.sleep()


if __name__ == '__main__':

    print("Geolocation node started")

    MARKER_ARRAY_TOPIC = rospy.get_param("/marker_array_topic")
    GNSS_TOPIC = rospy.get_param("/gnss_topic")
    BEARING_TOPIC = rospy.get_param("/bearing_topic")
    ELEVATION_TOPIC = rospy.get_param("/elevation_topic")

    rospy.init_node('geo_publisher', anonymous=True)
    rospy.Subscriber(MARKER_ARRAY_TOPIC, MarkerArray, marker_array_callback)
    rospy.Subscriber(GNSS_TOPIC, NavSatFix, gnss_callback)
    rospy.Subscriber(BEARING_TOPIC, Float32, bearing_callback)
    rospy.Subscriber(ELEVATION_TOPIC, Float32, elevation_callback)

    try:
        multi_track_publisher()
    except rospy.ROSInterruptException:
        pass
