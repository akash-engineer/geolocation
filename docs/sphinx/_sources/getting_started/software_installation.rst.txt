.. _Software Installation:

Software Installation
=====================
Parts of this guide are adapted from the Innovusion Falcon Prime LiDAR System
User Manual. Users should refer to the original manual for clarification
beyond this guide.

Dependencies
------------
The ``geolocation`` package depends on the following packages:

* rviz_satellite_
* innovusion_pointcloud
* object3d_detector

Install all dependencies before installing ``geolocation``. rviz_satellite is an
open-source piece of software, but the other two dependencies should be retrieved
from their respective developers. It is assumed that the system already has ROS
noetic installed. 

LiDAR Setup
-----------

Installing the Drivers
^^^^^^^^^^^^^^^^^^^^^^
This guide assumes you have already gotten the SDK files downloaded on your system.

The drivers for the Innovusion Falcon Prime are releases as ``.deb`` files. Select
the correct release for your system and install it.

The naming scheme for the releases is as follows:

``ros-[ROS DISTRO]-innovusion-driver-release-[RELEASE]-rc4-[ARCHITECTURE]-public.deb``

Install the drivers using dpkg.

.. code-block:: bash

    sudo dpkg -i <package.deb>


Default Network Config
^^^^^^^^^^^^^^^^^^^^^^
Before connecting to the LiDAR system, the network configuration of the computer/server 
needs to be changed such that they are on the same network segment as the LiDAR sensor.

Change the network settings to the following:

* IPv4 Method: ``Manual``
* IPv4 Address: ``172.168.1.1``
* Netmask: ``255.255.255.0``
* Gateway: ``[Leave Blank]``

.. image:: ../images/net_config.png
    :width: 600

Apply the changes after adjusting the configuration.

Changing the Network Config
^^^^^^^^^^^^^^^^^^^^^^^^^^^
By default, the IP address of the LiDAR sensor is ``172.168.1.10``. The IP address, Netmask
address and gateway address can be changed using the ``innovusion_lidar_util`` tool included
with the innovusion drivers and software packages.

The commands to make changes to the LiDAR's network defaults are:

* Unzip the SDK files

.. code-block:: bash

    tar -xzvf ..<package.tgz>

* Navigate to the utils folder

.. code-block:: bash

    cd ./<package>/apps/lidar_util

* Run the utility

.. code-block:: bash

    ./innovusion_lidar_util <ip of LIDAR> set_network <new_ip_address> <new_netmask_address> [new_gateway_address]

After the LiDAR sensor is powered off and on again, its network config will change.

object3d_detector Setup
-----------------------
object3d_detector is also released as a ``.deb`` file. It can be installed using dpkg.

.. code-block:: bash
    
    sudo dpkg -i <package.deb>


Geolocation Setup
-----------------
To install ``geolocation``, copy the entire package to your ``src`` folder in
your catkin workspace. Then, run ``catkin_make`` from your workspace root.

Source the ROS installation and workspace when done.

.. code-block::

    source /opt/ros/noetic/setup.bash

.. code-block::

    source <workspace root>/devel/setup.bash
    
.. _rviz_satellite: https://github.com/nobleo/rviz_satellite
