<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# compass\_conversions

This package contains utilities for converting between the various parametrizations of
[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html) messages.

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
[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html),
[QuaternionsStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/QuaternionStamped.html),
[PoseWithCovarianceStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html),
[Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html),
) and converts it to [Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html).

Some parameters might be needed for the conversion, and you can either supply them as arguments of the Subscriber,
or they can be autodetected from the topic name (if it follows the output of topic names module in this package).

### compass\_conversions::CompassFilter

A [message filter](https://wiki.ros.org/message_filters) that converts incoming
[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html) messages to a specified parametrization.
It can also handle a second input with [NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
messages to allow converting between different references. It also handles a third input with
[Int32](https://docs.ros.org/en/api/std_msgs/html/msg/Int32.html) messages to allow forcing a specific UTM zone.

Example usage:

```
message_filters::Subscriber azimuthInput(...);
message_filters::Subscriber fixInput(...);
compass_conversions::CompassFilter filter(log, nullptr, azimuthInput, fixInput,
  compass_msgs::Azimuth::UNIT_RAD, compass_msgs::Azimuth::ORIENTATION_ENU,
  compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC);
filter.registerCallback([](const compass_msgs::AzimuthConstPtr& msg) {
  ...  // Handle the data
});
```

### compass\_conversions/tf2\_compass\_msgs.h

When you include this file, `tf2::convert()` and similar functions will gain the ability to transform
[Azimuth](https://docs.ros.org/en/api/compass_msgs/html/msg/Azimuth.html) messages.

### compass\_conversions/topic\_names.h

Functions that assign topic names (or suffixes) to various parametrizations and representations of azimuths.
These unique topic name suffixes can be used to add important metadata to messages that do not carry them inside.

## Nodelets

### compass\_conversions/compass\_transformer

This nodelet subscribes to incoming azimuth messages (using the `UniversalAzimuthSubscriber` described above),
transforms them into a different parametrization (using `CompassFilter`), transforms them into a different TF frame
(using `tf2_compass_msgs.h`) and publishes them in the desired parametrization and representation (possibly using
`CompassConverter` to do the conversion).

The nodelet can also be launched as a standalone node using `rosrun compass_conversions compass_transformer`.

#### Subscribed topics:

- `~azimuth_in` (compass_msgs/Azimuth or geometry_msgs/QuaternionStamped or geometry_msgs/PoseWithCovarianceStamped
    or sensor_msgs/Imu): The input azimuth. The name of the topic (if you remap it) can be used to autodetect some
    metadata for the conversion.
 - `fix` (sensor_msgs/NavSatFix): GNSS fix messages that can be used to determine some parameters for the conversion.
 - `utm_zone` (std_msgs/Int32): Optional messages with forced UTM zone.
 - TF (only if `~target_frame` is nonempty)

#### Published topics:

 - `~azimuth_out` or `~azimuth_out/SUFFIX`: The transformed azimuth. If `~target_append_suffix` is true, the variant
                                            with topic name suffix will be used (e.g. `~azimuth_out/mag/enu/deg`).
                                            The type of the published message is determined by `~target_type`.

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
- `~subscribe_fix` (bool, default true): Whether to subscribe `fix` topic. In some cases, you don't need it.
- `~subscribe_utm` (bool, default true): Whether to subscribe `utm_zone` topic. It is fully optional.
- `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
                                                                    input messages (in case orientation cannot be
                                                                    derived either from message contents or topic
                                                                    name).
- `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
                                                                                   interpret input messages (in case
                                                                                   reference cannot be derived either
                                                                                   from message contents or topic
                                                                                   name).
- `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
                                               if variance cannot be determined from the input messages (e.g. for
                                               `QuaternionStamped`).
- `~strict` (bool, default true): If true, conversions between magnetic and geographic North will fail if the used
                                  magnetic model is used outside its declared bounds of validity (mostly year and
                                  altitude).
- All parameters consumed by `CompassConverter` (most interesting are `initial_lat`, `initial_lon`, that can relieve
  this nodelet from subscribing `fix` topic, if you know the approximate coordinates in advance).