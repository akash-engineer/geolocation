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
    