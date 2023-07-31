.. _tldr:

TL;DR
=====================
This page is a cheat sheet for all the most commonly used commands for
working with the LiDAR system.  It is intended to be a quick reference
for those who are already familiar with the system.

Starting the LiDAR system
-------------------------
1. Connect the LiDAR system to the computer via Ethernet.
2. Power on the LiDAR system.
3. Change the network profile to `LiDAR Connection`
4. Open a terminal and run the command:

.. code-block:: bash

    start_lidar

If any red text appears in the terminal before the Rviz window opens, then
one of the programs has failed to start properly. Refer to the troubleshooting
section below.

Stopping the LiDAR system
-------------------------
1. Return to the terminal window where the LiDAR system was started.
2. Press `Ctrl+C` to stop the LiDAR system.
3. Wait for all the programs to close. The last line of the terminal should
   read `done`.

Troubleshooting
---------------

LiDAR software completely fails to start or crashes immediately
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is likely caused due to a wrongly configured network profile.  Make sure
that the network profile is set to `LiDAR Connection` and that the IP address
is set to `192.168.1.1`.

Alternatively, the LiDAR system may not be connected to the computer via
Ethernet.

ERROR: cannot launch node of type [object3d_detector/object3d_detector]: object3d_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This indicates a failure to launch the object3d_detector node. This is
usually caused by a missing dependency.  Make sure that the `object3d_detector`
package is installed.

1. Open a terminal and run the commands:
    .. code-block:: bash

        rosSource
        wsSource
        rospack list-names

2. Check that `object3d_detector` is listed in the output of the last command.
3. If it is not listed, then reinstall the package:

    .. code-block:: bash

        cd <PATH TO DEB FILE>
        sudo dpkg -i object3d_detector_<VERSION>.deb

[rospy.core][INFO] <DATE AND TIME>: signal_shutdown [atexit]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This indicates that one of the python scripts has crashed or
received a shutdown signal. This is usually caused by a bugged
python script.

Check the logs for the python script that crashed.  The logs are
located in the `~/.ros/log` directory.  The log file will be named
after the node that crashed.

The script must be fixed before the full LiDAR system can run properly.
