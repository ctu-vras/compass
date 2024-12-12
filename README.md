<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# Compass stack

This collection of packages provides support for working with azimuths in ROS.

## Packages

- [compass_msgs](compass_msgs): The message definitions.
- [compass_conversions](compass_conversions): Helpers for converting between different representations of azimuths.
- [magnetic_model](magnetic_model): ROS bindings for World Magnetic Model.
- [magnetometer_compass](magnetometer_compass): Support and ROS nodes for extracting azimuths from 3-axis magnetometers.
- [magnetometer_pipeline](magnetometer_pipeline): Calibration and removing of magnetometer bias.

## Definitions

**ENU** frame is the [standard orientation used in ROS](https://www.ros.org/reps/rep-0103.html). The abbreviation means
East-North-Up and corresponds to the meaning of vector components X, Y and Z. A zero azimuth points towards East and it
increases counter-clockwise.

**NED** frame is the "intuitive" North-East-Down orientation where the zero azimuth points to North and increases
clockwise, just as you are used to when using a compass.

**Magnetic azimuth** is the angle between Earth's magnetic North (or East in ENU frame) and a specified direction.

**True azimuth** (also called geographic, map or geodetic North) is the angle between Earth's geographic North (or East
in ENU frame) and a specified direction.

**UTM azimuth** (also called grid azimuth) is the angle between UTM North (or East in ENU frame) and a specified
direction. [UTM](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) is a planar projection
of Earth's surface onto predefined rectangles, which yields a Cartesian coordinate system. The Earth is divided into
several stripes which are unrolled into a plane to form the UTM grid. These stripes are called **UTM zones**. Each UTM
zone is 6 degrees of longitude wide, but it is considered valid in a slightly larger area, approximately 100 km
outside its precise bounds. This allows sticking to a single UTM zone to prevent zone switching when moving close to
the boundary of two zones.

The difference between magnetic and true North is called **magnetic declination**. Its values are location- and
time-dependent and they are approximated by the
[World Magnetic Model](https://www.ncei.noaa.gov/products/world-magnetic-model).

The difference between true North and grid North is called **grid convergence**. Its values are only location-dependent
and do not differ in time. The values also depend on the chosen UTM zone.

Although [ROS specifies that all angular values should be expressed in radians](https://www.ros.org/reps/rep-0103.html),
the usage of degrees in geography is so common that
[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html) messages support both radians and degrees.

For more information, see https://www.drillingformulas.com/magnetic-declination-and-grid-convergent-and-their-applications-in-directional-drilling/
or https://en.wikipedia.org/wiki/Azimuth .
