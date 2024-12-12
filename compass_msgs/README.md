<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# compass\_msgs

Messages for use with a compass.

## Azimuth

[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html) message carries information about an azimuth.
As there are multiple possibilities related to the azimuth meaning, it has 3 metadata fields that exactly specify the
meaning of each message:

- `unit`:
  - `UNIT_DEG`: `azimuth` field is in degrees and `variance` in degrees^2
  - `UNIT_RAD`: `azimuth` field is in radians and `variance` in rad^2
- `orientation`
  - `ORIENTATION_ENU`: The azimuth is measured in East-North-Up frame. Azimuth 0 thus points to the East and increases
      counter-clockwise.
  - `ORIENTATION_NED`: The azimuth is measured in North-East-Down frame. Azimuth 0 thus points to the North and
      increases clockwise.
- `reference`: Which North reference is used.
  - `REFERENCE_MAGNETIC`: Magnetic North.
  - `REFERENCE_GEOGRAPHIC`: Geographic North.
  - `REFERENCE_UTM`: UTM grid North.