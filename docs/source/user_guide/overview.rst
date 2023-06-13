.. _Overview:

Overview
========
This page covers the main features of the ``geolocation`` package.

GCS Coordinates
---------------
The package automatically calculates the GCS coordinates of all identified targets
based on their position relative to the sensor, and the sensor's known GCS coordinates.
The calculated coordinates are not shown in the rviz visualisation, but are available
in the ``tracker<X>/gnss`` topic, where <X> is the tracker number. 

Note that the tracker number may not be consistent at this time, and the ID of each
tracked target may swap between trackers. This will (hopefully) be fixed in a future
release of the ``object3d_detector``.

THE GCS coordinates are published as a ``sensor_msgs/NavSatFix`` message, but only
the ``header``, ``latitude``, ``longitude`` and ``altitude`` fields are populated.

Point Cloud Visualisation
-------------------------
The package provides a visualisation of the point cloud data from the sensor, and
the detected targets. The visualisation is provided by the ``rviz`` package, and
is launched by defualt with all of the included launch files.

The visualisation is split into two parts, the point cloud data, and the detected
targets. The point cloud data is shown as a ``PointCloud2`` message, and the targets
are shown as ``MarkerArray`` messages.

The included rviz configurations are set up to anchor the camera on the ``base_link``
frame. A transform to properly align the point cloud data with the ``map`` frame is
provided by the ``base_tf2_broadcaster`` as the Innovusion Falcon Prime publishes
its point cloud data with differently mapped axes.

Visualisation Controls
^^^^^^^^^^^^^^^^^^^^^^
The camera controls for the rviz visualisation are as follows:

* ``Left-Click and Drag`` - Change the direction the camera is facing (i.e. Pan and Tilt)
* ``Right-Click and Drag`` - Change the zoom level of the camera
* ``Middle-Click and Drag`` - Change the position of the camera (i.e. Move the camera)

The position of the camera can be reset to the base_link by clicking the ``Zero`` button
at the top of the right hand panel. This does not reset the camera orientation.

Target Tracking Visualisation
-----------------------------
Targets identified by the ``object3d_detector`` are shown as ``MarkerArray`` messages
in the rviz visualisation. The markers are shown as green bounding boxes, with the
xyz coordinates of the target shown in the bottom right corner of the bounding box.

The markers are shown and anchored to the ``innovusion`` frame, which is the frame
that the LiDAR sensor publishes its point cloud data in. This frame is not aligned
with any other frames and requires the base_tf2_broadcaster to align it with the
``map`` frame.

The markers' properties like colour and size are not configurable from the rviz GUI,
so targets at longer ranges may be difficult to see without zooming in on the target.
This is a feature of the ``object3d_detector`` package, and is not configurable from
the ``geolocation`` package. (This is a feature to be requested. Refer to :ref:`Feature
Requests` for more info.)

Web Interface (In Development)
------------------------------
This package implements the ``rosbridge`` package to allow web apps or other javascript
programs to access its topics and parameters. The web interface is still in development,
but a basic interface is available in the ``map_view.html`` file. 

The rosbridge package is not included in the ``geolocation`` package, and must be
installed separately. The ``rosbridge`` package can be installed using the following
command:

.. code-block:: bash

    sudo apt install ros-<distro>-rosbridge-server

Where ``<distro>`` is the ROS distribution you are using. For example, if you are
using ROS Noetic, the command would be:

.. code-block:: bash

    sudo apt install ros-noetic-rosbridge-server

Once the ``rosbridge`` package is installed, ``geolocation`` can be launched with
the rosbridge server using launch files. A live envirmonemt launch file has not
been implemented yet, but a demo launch file is available in the ``launch`` folder.

The demo launch file can be launched using the following command:

.. code-block:: bash

    roslaunch geolocation demo_bridge.launch

This launch file is identical to the ``demo.launch`` file, but also launches the
``rosbridge`` server. The basic web UI can be accessed by opening the
``map_view.html`` file in a web browser.
