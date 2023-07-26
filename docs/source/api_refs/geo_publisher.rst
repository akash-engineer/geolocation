geo_publisher.py
================
This node subscribes to the gnss, heading, elevation and marker array topics
and publishes the gnss coordinates of each marker to the /tracker<X>/gnss topic
where <X> is the marker id.

The node caluclates the gnss coordinates of each marker based on the marker's
position relative to the sensor's position and orientation (assuming the sensor
is at the frame origin).

Autofunctions temporarily removed due to OOP restructuring.
