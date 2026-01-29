<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# compass\_interfaces

Messages for use with a compass.

## Azimuth

[Azimuth](https://docs.ros.org/en/rolling/p/compass_interfaces/msg/Azimuth.html) message carries information about an azimuth.
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

## ROS 2 Build status

| Distro | Source Ubuntu | Source RHEL | Ubuntu amd64 | Ubuntu arm64 | RHEL amd64 |
|--------|---------------|-------------|--------------|--------------|------------|
| Jazzy  | [![Jsrc_uN](https://build.ros2.org/job/Jsrc_uN__compass_interfaces__ubuntu_noble__source/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jsrc_uN__compass_interfaces__ubuntu_noble__source)  | [![Jsrc_el9](https://build.ros2.org/job/Jsrc_el9__compass_interfaces__rhel_9__source/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Jsrc_el9__compass_interfaces__rhel_9__source) | [![Jbin_uN64](https://build.ros2.org/job/Jbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary) [![jazzy default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_default.html?q=compass_interfaces) | [![Jbin_uNv8](https://build.ros2.org/job/Jbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Jbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary) [![jazzy unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_unv8.html?q=compass_interfaces) | [![Jbin_rhel](https://build.ros2.org/job/Jbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Jbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary) [![jazzy rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_jazzy_rhel.html?q=compass_interfaces)|
| Kilted  | [![Ksrc_uN](https://build.ros2.org/job/Ksrc_uN__compass_interfaces__ubuntu_noble__source/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Ksrc_uN__compass_interfaces__ubuntu_noble__source)  | [![Ksrc_el9](https://build.ros2.org/job/Ksrc_el9__compass_interfaces__rhel_9__source/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Ksrc_el9__compass_interfaces__rhel_9__source) | [![Kbin_uN64](https://build.ros2.org/job/Kbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Kbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary) [![jazzy default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_default.html?q=compass_interfaces) | [![Kbin_uNv8](https://build.ros2.org/job/Kbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Kbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary) [![jazzy unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_unv8.html?q=compass_interfaces) | [![Kbin_rhel](https://build.ros2.org/job/Kbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Kbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary) [![jazzy rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_kilted_rhel.html?q=compass_interfaces)|
| Rolling  | [![Rsrc_uN](https://build.ros2.org/job/Rsrc_uN__compass_interfaces__ubuntu_noble__source/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Rsrc_uN__compass_interfaces__ubuntu_noble__source)  | [![Rsrc_el9](https://build.ros2.org/job/Rsrc_el9__compass_interfaces__rhel_9__source/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Rsrc_el9__compass_interfaces__rhel_9__source) | [![Rbin_uN64](https://build.ros2.org/job/Rbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Rbin_uN64__compass_interfaces__ubuntu_noble_amd64__binary) [![jazzy default release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_default.html?q=compass_interfaces) | [![Rbin_uNv8](https://build.ros2.org/job/Rbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary/badge/icon?style=flat&subject=24.04)](https://build.ros2.org/job/Rbin_unv8_uNv8__compass_interfaces__ubuntu_noble_arm64__binary) [![jazzy unv8 release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_unv8.html?q=compass_interfaces) | [![Rbin_rhel](https://build.ros2.org/job/Rbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary/badge/icon?style=flat&subject=RHEL%209)](https://build.ros2.org/job/Rbin_rhel_el964__compass_interfaces__rhel_9_x86_64__binary) [![jazzy rhel release status](https://img.shields.io/badge/release-status-blue)](https://repo.ros2.org/status_page/ros_rolling_rhel.html?q=compass_interfaces)|
