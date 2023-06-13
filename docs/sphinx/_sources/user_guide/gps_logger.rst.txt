.. _Using a GPS Logger:

Using a GPS Logger
==================
The ``geolocation`` package includes a few programs that allow for gps data
to be added to a rosbag file after it has been recorded.  This is useful
for when you have a rosbag file that was recorded without gps data, but you
want to add it later.  The ``geolocation`` package was designed to work with
the Aaronia GPS Logger, but it should work with any csv file that has the
required format.

.. _Aaronia GPS Logger:

Aaronia GPS Logger
------------------
The Aaronia GPS Logger is a small device that is intended to be used alongside
AAronia's spectrum analysers, but can still be used as a standalone device. Its
software provided by Aaronia only runs on Windows, so data streaming is not
supported on Linux devices. The logger can still be used to record data to its
removable microSD card, which can then be read on a Linux device.

The GPS Logger uses the text-based NMEA 0183 syntax for communication, including
the standard GPRMC and GPGGA sentences. Commands and sensor-specific replies use
the PAAG vendor suffix. In general a command has the form

``$PAAG,<command>,<parameters><CR><LF>``

and replies use

``$<replycode>,<parameters>*<checksum><CR><LF>``

where CR and LF are ASCII codes 13 and 10, and checksum is a 2 character 
hexadecimal representation of the XOR combination of all characters between the
$ and the * (as defined by the NMEA protocol).

For more information, refer to the `Aaronia GPS Logger Programmer Guide <https://dev.aaronia-shop.com/downloads/gps/manuals/gps_logger_programming_guide_en.pdf>`_.

.. _Convert Aaronia GPS Logger data:

Converting the Aaronia GPS Logger data
--------------------------------------
The Aaronia GPS Logger records raw sensor data in NMEA sentences, which is not
directly compatible with ROS. Aaronia's software can convert the data to a csv
file, but the software only runs on Windows. At this time, this is the most
reliable way to convert the raw sensor data to useful coordinate and orientation
data.

Aaronia's software can be downloaded from their `downloads <https://aaronia.com/downloads/#gps_logger_software/>`_
page.

.. _bag_csv_merge:

bag_csv_merge
-------------
The ``bag_csv_merge`` program is used to merge a csv file with a rosbag. The CSV file
must contain the following headers:

* ``date`` - The date in the format ``YYYY-MM-DD``
* ``time(local time)`` - The local time in the format ``HH:MM:SS``
* ``altitude (m)`` - The altitude in meters
* ``latitude`` - The latitude in degrees
* ``longitude`` - The longitude in degrees
* ``tilt (deg)`` - The tilt in degrees
* ``roll (deg)`` - The roll in degrees
* ``compass (deg)`` - The compass heading in degrees

The program will convert the data between the start and end times of the rosbag into
ros messages of the relevant type and add them to the rosbag. The program will
automatically convert the data in degrees to raidans. 

The program can be run as follows::

    rosrun geolocation bag_csv_merge <ROSBAG_PATH> <CSV_PATH> <OUTPUT_PATH>

.. _bag_gps_merge:

bag_gps_merge
-------------
**This method has been depreciated in favor of the** ``bag_csv_merge`` **method.**

The ``bag_gps_merge`` program is used to merge a raw gps data file with a rosbag.
The raw gps data file must be in the format described above.  The program will
convert and NMEA sentences between the start and end times of the rosbag into
ros messages of the relevant type and add them to the rosbag.

At the moment, the program only supports the GPRMC, GPGGA and PAAG,DATA,C sentences.

The program can be run as follows::

    rosrun geolocation bag_gps_merge <ROSBAG_PATH> <GPS_DATA_PATH> <OUTPUT_PATH>
