.. _Parameters:

Parameters
==========
These parameters are saved in the ``/config/`` folder and are
automatically loaded when the package starts. They can be changed
manually by editing the files.

.. _ros_params:

ros_params.yaml
---------------
Most of the parameters are loaded from the ``ros_params.yaml`` file.

``rate``
    Sets the rate at which the main loop runs.

    Default: ``30``


``ground_height``
    Sets the ground height at the sensor's position. This is used to 
    offset the altitude measurements as most locations are significantly
    away from sea level.

    Future plans include creating a script to automatically set this
    using an external mapping service.

    Default: ``8.0``


``innovusion_tf_r``
    Sets the roll of the Innovusion frame relative to the base_frame.
    This is required as the point cloud from the LiDAR sensor is
    published in the Innovusion frame, which has its axes oriented 
    differently.

    Default: ``rad(-pi/2)``


``innovusion_tf_p``
    Sets the pitch of the Innovusion frame relative to the base_frame.
    See ``innovusion_tf_r`` for more details.

    Default: ``rad(0.0)``


``innovusion_tf_y``
    Sets the yaw of the Innovusion frame relative to the base_frame.
    See ``innovusion_tf_r`` for more details.

    Default: ``rad(-pi/2)``

.. _topic_names:

topic_names.yaml
----------------
All topic names are saved in this file for easier renaming of topics as
necessary.

``gnss_topic``
    The topic name for the GNSS data of the LiDAR sensor.

    Default: ``/sensor_pose/gnss``


``elevation_topic``
    The topic name for the elevation data of the LiDAR sensor.

    Default: ``/sensor_pose/elevation``


``bearing_topic``
    The topic name for the bearing data of the LiDAR sensor.

    Default: ``/sensor_pose/bearing``


``roll_topic``
    The topic name for the roll data of the LiDAR sensor.

    Default: ``/sensor_pose/roll``


``marker_array_topic``
    The topic name for the marker array data of the LiDAR sensor.

    Default: ``/object3d_detector/markers``

