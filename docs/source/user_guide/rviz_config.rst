.. _Rviz Config:

Rviz Config
===========
All of the built-in visualisation in this package is done using Rviz. This page
covers how to change the various settings in rviz to best suit your needs.

Displays
--------
The left hand side of the rviz window is the Displays panel. This is where you
can enable and disable the various visualisations. The most important ones are
the ``PointCloud2`` and ``MarkerArray`` displays. The ``PointCloud2`` display
shows the point cloud data from the LiDAR sensor, and the ``MarkerArray`` display
shows the detected objects.

Global Options
^^^^^^^^^^^^^^
These options affect all other visualisations in rviz.

* Fixed Frame:
    This should be set to ``innovusion`` when running with a live LiDAR sensor,
    or ``map`` when playing back a bag file.

    Changing this setting will likely cause the PointCloud2 display to stop
    working, as the small delay caused by the transform will cause the display
    to drop the data with a ``Message too old`` error.

* Background Color:
    This can be changed to suit your preferences. The default is a dark grey (48,48,48).

* Frame Rate:
    This can be changed to suit your preferences. The default is 30.

    Note that this setting changes the `target` frame rate of `rviz only`. It
    does not affect the rate at which the LiDAR sensor publishes data, or the
    rate at which the visualisations are updated.

    By default, the visualisations are updated at a rate of 30Hz, but this can
    be changed in the parameter config files. Refer to the :ref:`params <Parameters>`
    page for more info. Setting the rviz frame rate to a value higher than the
    visualisation rate will not cause the visualisations to update faster.

* Default Light:
    This should be kept as ``true``. Disabling this option will cause objects like
    markers to lose all colour.


Global Status
^^^^^^^^^^^^^
The Global Status panel shows the status of the various topics that rviz is
subscribed to. Its icon should show a green tick and it should have only
``OK`` messages. If it shows a red cross or a yellow warning sign, then there
is a problem with one of the topics and should be addressed.

AerialMapDisplay
^^^^^^^^^^^^^^^^
This display controls the map that is rendered on the ground plane in rviz.

* Status:
    This should show ``OK`` if there is a GNSS topic being published to. If
    there is no GNSS topic, then the map will not be rendered and this should
    return an error or warning status.

* Topic:
    This should be set to the GNSS topic that is being published to. The default
    is ``/sensor_pose/gnss``.

* Map transform type:
    This should be set to ``NavSatFix Massages and Map Frame``. The rviz_satellite
    package also supports using UTM coordinates, but this package does not, so
    this option should not be used.

* Map frame:
    This should be set to ``map``. This is the frame that the map will be
    rendered in.

TF
^^^^
This display shows the transform tree. It is useful for debugging, but should not
need to be adjusted for normal operation.

PointCloud2
^^^^^^^^^^^
This display shows the point cloud data from the LiDAR sensor. It is one of
the most important displays in rviz, and should be enabled.

MarkerArray
^^^^^^^^^^^
This display shows the detected objects. It is one of the most important
displays in rviz, and should be enabled.


Views
-----
The right hand side of the rviz window is the Views panel. This is where you
can change the camera settings. This panel is hidden by default, but can be
opened by clicking the arrow on the right hand side of the window.

Most of the settings in this window can be adjusted by clicking and dragging
the mouse in the main rviz window. The following settings are the most important:

* Type:
    This should be set to ``FrameAligned``. The fixed frame must be set to the
    ``innovusion`` frame for the PointCloud2 to be visualised, but it is not
    aligned with the world, so this setting should be used to align the camera
    with the world.

* Near Clip Distance:
    This should be set to ``0.01``. Setting this to a value higher than 0.1 will
    cause objects that are close to the camera to be clipped.

* Target Frame:
    This should be set to ``base_link``. This is the frame that the camera will
    be aligned with by default. A transform has already been applied from the
    ``base_link`` frame to the ``innovusion`` frame, so this setting should not
    be changed.