.. _GNSS Coordinates:

Obtaining GNSS Coordinates
==========================
This program uses the GCS (aka Geodetic) coordinate reference system for its geographic
coordinates. All units are in Decimal Degrees.

The LiDAR sensor provides accurate spacial coordinates of each point in the point cloud,
so they can be used to calculate the distance and angle of the target from the sensor.

Given the short operating distance of the LiDAR sensor, all distances can be calculated
assuming a flat earth plane (i.e. no curvature). This is a reasonable assumption for
distances less than 10km.

The math
--------
In order to calculate the GCS coordinates of the target, the nadir distance (distance
following the earth's curvature) and absolute heading of the target relative to the
sensor must be known.

To make these calculations easier, the frame of the LiDAR sensor is transformed such that
it aligns with the world's frame of reference. This is done by rotating the LiDAR sensor's
frame of reference by the sensor's heading and absoulte heading. 

After the transformation, the nadir distance and absolute heading of the target can be
calculated using basic trigonometry.

The nadir distance can be caluclated using the Pythagorean Theorem on the x and y
coordinates of the target since the x-y plane is now aligned with the ground plane:

.. math::
    d_{nadir}=\sqrt{x^2 + y^2}

The absolute heading of the target can be calculated using the arctangent of the x and y
coordinates of the target since the x-y plane is now aligned with the ground plane:

.. math::
    h_{abs}=\arctan{(\frac{y}{x})}

The GSC coordinates are then calculated using the following formulae:

.. math::
    lat_{target}=\arcsin{((\sin{lat_{start} * \cos{\frac{d_{nadir}}{r_{earth}}}}) + (\cos{lat_{start}} * \sin{\frac{d_{nadir}}{r_{earth}}} * \cos{h}))}

.. math::
    lon_{target}=lon_{start} + \arctan{(\frac{\sin{h} * \sin{\frac{d_{nadir}}{r_{earth}}}}{\cos{lat_{start}} * \cos{\frac{d_{nadir}}{r_{earth}}} - \sin{lat_{start}} * \sin{lat_{target}}})}