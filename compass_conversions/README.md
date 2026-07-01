<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# compass\_conversions

This package contains utilities for converting between the various parametrizations of
[Azimuth](https://docs.ros.org/en/api/compass_interfaces/html/msg/Azimuth.html) messages.

See [readme of the compass stack](../README.md) for definitions of the terms used in this readme.

## C++ Libraries

### compass\_conversions::CompassConverter

Helper for direct conversions between parametrizations.

Conversions between different references (e.g. Mag North to True North) require a
[NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) with the corresponding absolute pose.

Conversions involving UTM can receive a [Int32](https://docs.ros.org/en/api/std_msgs/html/msg/Int32.html) parameter
that forces the use of a specific UTM zone instead of the default one.

The converter accepts several parameters that modify its behavior:

- `magnetic_declination` (double, radians, optional): If set, forces this value of magnetic declination.
- `utm_grid_convergence` (double, radians, optional): If set, forces this value of UTM grid convergence.
- `magnetic_models_path` (string, default `"$PACKAGE/data/magnetic"`): Path where WMM magnetic models can be found.
- `magnetic_model` (string, optional): If set, forces using the given WMM model instead of determining the proper
    one by year. Example value is "wmm2020".
- `utm_zone` (int, optional): If set, forces using this UTM zone instead of determining the proper one.
- `keep_utm_zone` (bool, default true): If true, the first automatically determined UTM zone will be used for all future
    conversions.
- `initial_lat` (double, degrees, optional): If set, use this latitude before the first navsat pose is received.
- `initial_lon` (double, degrees, optional): If set, use this longitude before the first navsat pose is received.
- `initial_alt` (double, meters, optional): If set, use this altitude before the first navsat pose is received.

### compass\_conversions::UniversalAzimuthSubscriber

A [message filter](https://wiki.ros.org/message_filters) `Subscriber` that subscribes any of the supported azimuth
representations (
[Azimuth](https://docs.ros.org/en/api/compass_interfaces/html/msg/Azimuth.html),
[QuaternionStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/QuaternionStamped.html),
[PoseWithCovarianceStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/pose_with_covariance_stamped.hpptml),
[Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html),
) and converts it to [Azimuth](https://docs.ros.org/en/api/compass_interfaces/html/msg/Azimuth.html).

Some parameters might be needed for the conversion, and you can either supply them as arguments of the Subscriber,
or they can be autodetected from the topic name (if it follows the output of topic names module in this package).

### compass\_conversions::CompassFilter

A [message filter](https://wiki.ros.org/message_filters) that converts incoming
[Azimuth](https://docs.ros.org/en/api/compass_interfaces/html/msg/Azimuth.html) messages to a specified parametrization.
It can also handle a second input with [NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
messages to allow converting between different references. It also handles a third input with
[Int32](https://docs.ros.org/en/api/std_msgs/html/msg/Int32.html) messages to allow forcing a specific UTM zone.

Example usage:

```
message_filters::Subscriber azimuthInput(...);
message_filters::Subscriber fixInput(...);
compass_conversions::CompassFilter filter(log, nullptr, azimuthInput, fixInput,
  compass_interfaces::msg::Azimuth::UNIT_RAD, compass_interfaces::msg::Azimuth::ORIENTATION_ENU,
  compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC);
filter.registerCallback([](const compass_interfaces::msg::AzimuthConstSharedPtr& msg) {
  ...  // Handle the data
});
```

### compass\_conversions/tf2\_compass\_msgs.h

When you include this file, `tf2::convert()` and similar functions will gain the ability to transform
[Azimuth](https://docs.ros.org/en/api/compass_interfaces/html/msg/Azimuth.html) messages.

### compass\_conversions/topic\_names.h

Functions that assign topic names (or suffixes) to various parametrizations and representations of azimuths.
These unique topic name suffixes can be used to add important metadata to messages that do not carry them inside.

## Component nodes

### Node compass\_transformer and component node compass\_conversions::CompassTransformerNodelet

This component node subscribes to incoming azimuth messages (using the `UniversalAzimuthSubscriber` described above),
transforms them into a different parametrization (using `CompassFilter`), transforms them into a different TF frame
(using `tf2_compass_msgs.h`) and publishes them in the desired parametrization and representation (possibly using
`CompassConverter` to do the conversion).

The component can also be launched as a standalone node using `ros2 run compass_conversions compass_transformer`.

#### Subscribed topics:

- `~azimuth_in` (compass_interfaces/Azimuth): The input azimuth. The name of the topic (if you remap it)
    can be used to autodetect some metadata for the conversion.
- `~azimuth_in/imu` (sensor_msgs/Imu): The input azimuth in IMU format. The name of the topic (if you remap it)
    can be used to autodetect some metadata for the conversion.
- `~azimuth_in/pose` (geometry_msgs/PoseWithCovarianceStamped): The input azimuth in IMU format. The name of the
    topic (if you remap it) can be used to autodetect some metadata for the conversion.
- `~azimuth_in/quat` (geometry_msgs/QuaternionStamped): The input azimuth in IMU format. The name of the topic
    (if you remap it) can be used to autodetect some metadata for the conversion.
 - `fix` (sensor_msgs/NavSatFix): GNSS fix messages that can be used to determine some parameters for the conversion.
 - `utm_zone` (std_msgs/Int32): Optional messages with forced UTM zone.
 - TF (only if `~target_frame` is nonempty)

#### Published topics:

 - `~azimuth_out` or `~azimuth_out/SUFFIX`: The transformed azimuth. If `~target_append_suffix` is true, the variant
    with topic name suffix will be used (e.g. `~azimuth_out/mag/enu/deg`). The type of the published message is
    determined by `~target_type`.

#### Parameters:

- `~queue_size` (int, default 10): Queue size for the subscribers and publishers.
- `~target_unit` (str, 'deg' or 'rad', default: no change): Angular unit to be used in the transformed messages.
- `~target_orientation` (str, 'enu' or 'ned', default: no change): ENU or NED orientation to be used in the
    transformed messages.
- `~target_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used in the
    transformed messages.
- `~target_type` (str, 'azimuth', 'quaternion', 'pose' or 'imu', default 'azimuth'): The Type of output messages.
- `~target_append_suffix` (bool, default false): If true, the output topic will be suffixed with a metadata-based
    string.
- `~target_frame` (str, default: no change): TF frame to transform the messages to. Please note that frames that are
    too "titled" from gravity will not make much sense.
- `~out_frame_id` (str, default: no change): If nonempty, the `frame_id` of the transformed messages will be substituted
    by this value. Different from `~target_frame`, no geometrical transformations are done.
- `~subscribe_fix` (bool, default true): Whether to subscribe `fix` topic. In some cases, you don't need it.
- `~subscribe_utm` (bool, default true): Whether to subscribe `utm_zone` topic. It is fully optional.
- `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
    input messages (in case orientation cannot be derived either from message contents or topic name).
- `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
    interpret input messages (in case reference cannot be derived either from message contents or topic name).
- `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
    if variance cannot be determined from the input messages (e.g. for `QuaternionStamped`).
- `~strict` (bool, default true): If true, conversions between magnetic and geographic North will fail if the used
    magnetic model is used outside its declared bounds of validity (mostly year and altitude).
- All parameters consumed by `CompassConverter` (most interesting are `initial_lat`, `initial_lon`, that can relieve
    this nodelet from subscribing `fix` topic, if you know the approximate coordinates in advance).

## ROS 2 Build status

| Distro | Source Ubuntu | Source RHEL | Ubuntu amd64 | Ubuntu arm64 | RHEL amd64 |
|--------|---------------|-------------|--------------|--------------|------------|
| Jazzy  | [![Jsrc_uN](https://build.ros2.org/job/Jsrc_uN__compass_conversions__ubuntu_noble__source/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jsrc_uN__compass_conversions__ubuntu_noble__source)  | [![Jsrc_el9](https://build.ros2.org/job/Jsrc_el9__compass_conversions__rhel_9__source/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Jsrc_el9__compass_conversions__rhel_9__source) | [![Jbin_uN64](https://build.ros2.org/job/Jbin_uN64__compass_conversions__ubuntu_noble_amd64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jbin_uN64__compass_conversions__ubuntu_noble_amd64__binary) [![jazzy default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_default.html?q=compass_conversions) | [![Jbin_uNv8](https://build.ros2.org/job/Jbin_unv8_uNv8__compass_conversions__ubuntu_noble_arm64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jbin_unv8_uNv8__compass_conversions__ubuntu_noble_arm64__binary) [![jazzy unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_unv8.html?q=compass_conversions) | [![Jbin_rhel](https://build.ros2.org/job/Jbin_rhel_el964__compass_conversions__rhel_9_x86_64__binary/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Jbin_rhel_el964__compass_conversions__rhel_9_x86_64__binary) [![jazzy rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_rhel.html?q=compass_conversions)|
| Kilted  | [![Ksrc_uN](https://build.ros2.org/job/Ksrc_uN__compass_conversions__ubuntu_noble__source/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Ksrc_uN__compass_conversions__ubuntu_noble__source)  | [![Ksrc_el9](https://build.ros2.org/job/Ksrc_el9__compass_conversions__rhel_9__source/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Ksrc_el9__compass_conversions__rhel_9__source) | [![Kbin_uN64](https://build.ros2.org/job/Kbin_uN64__compass_conversions__ubuntu_noble_amd64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Kbin_uN64__compass_conversions__ubuntu_noble_amd64__binary) [![jazzy default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_default.html?q=compass_conversions) | [![Kbin_uNv8](https://build.ros2.org/job/Kbin_unv8_uNv8__compass_conversions__ubuntu_noble_arm64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Kbin_unv8_uNv8__compass_conversions__ubuntu_noble_arm64__binary) [![jazzy unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_unv8.html?q=compass_conversions) | [![Kbin_rhel](https://build.ros2.org/job/Kbin_rhel_el964__compass_conversions__rhel_9_x86_64__binary/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Kbin_rhel_el964__compass_conversions__rhel_9_x86_64__binary) [![jazzy rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_rhel.html?q=compass_conversions)|
| Lyrical | [![Lsrc_uR](https://build.ros2.org/job/Lsrc_uR__compass_conversions__ubuntu_resolute__source/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Lsrc_uR__compass_conversions__ubuntu_resolute__source) | [![Lsrc_el10](https://build.ros2.org/job/Lsrc_el10__compass_conversions__rhel_10__source/badge/icon?style=flat&subject=RHEL%2010)](https://build.ros2.org/job/Lsrc_el10__compass_conversions__rhel_10__source) | [![Lbin_uR64](https://build.ros2.org/job/Lbin_uR64__compass_conversions__ubuntu_resolute_amd64__binary/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Lbin_uR64__compass_conversions__ubuntu_resolute_amd64__binary) [![lyrical default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_lyrical_default.html?q=compass_conversions) | [![Lbin_uRv8](https://build.ros2.org/job/Lbin_unv8_uRv8__compass_conversions__ubuntu_resolute_arm64__binary/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Lbin_unv8_uRv8__compass_conversions__ubuntu_resolute_arm64__binary) [![lyrical unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_lyrical_unv8.html?q=compass_conversions) | [![Lbin_rhel](https://build.ros2.org/job/Lbin_rhel_el1064__compass_conversions__rhel_10_x86_64__binary/badge/icon?style=flat&subject=RHEL%2010)](https://build.ros2.org/job/Lbin_rhel_el1064__compass_conversions__rhel_10_x86_64__binary) [![lyrical rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_lyrical_rhel.html?q=compass_conversions) |
| Rolling | [![Rsrc_uR](https://build.ros2.org/job/Rsrc_uR__compass_conversions__ubuntu_resolute__source/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Rsrc_uR__compass_conversions__ubuntu_resolute__source) | [![Rsrc_el9](https://build.ros2.org/job/Rsrc_el10__compass_conversions__rhel_10__source/badge/icon?style=flat&subject=RHEL%2010)](https://build.ros2.org/job/Rsrc_el10__compass_conversions__rhel_10__source)  | [![Rbin_uR64](https://build.ros2.org/job/Rbin_uR64__compass_conversions__ubuntu_resolute_amd64__binary/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Rbin_uR64__compass_conversions__ubuntu_resolute_amd64__binary) [![rolling default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_default.html?q=compass_conversions) | [![Rbin_uRv8](https://build.ros2.org/job/Rbin_unv8_uRv8__compass_conversions__ubuntu_resolute_arm64__binary/badge/icon?style=flat&subject=26.04)](https://build.ros2.org/job/Rbin_unv8_uRv8__compass_conversions__ubuntu_resolute_arm64__binary) [![rolling unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_unv8.html?q=compass_conversions) | [![Rbin_rhel](https://build.ros2.org/job/Rbin_rhel_el1064__compass_conversions__rhel_10_x86_64__binary/badge/icon?style=flat&subject=RHEL%2010)](https://build.ros2.org/job/Rbin_rhel_el1064__compass_conversions__rhel_10_x86_64__binary) [![rolling rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_rhel.html?q=compass_conversions) |