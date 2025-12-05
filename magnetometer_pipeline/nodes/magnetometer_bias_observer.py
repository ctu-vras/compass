#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Magnetometer bias estimation node."""

import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_services_default, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


class MagBiasObserver(Node):

    def __init__(self):
        super().__init__('magnetometer_bias_observer')

        self.declare_parameters(namespace='', parameters=[
            ('measuring_time', 30.),
            ('2d_mode', True),
            ('2d_mode_ignore_axis', ''),
            ('load_from_params', False),
            ('magnetometer_bias_x', 0.0),
            ('magnetometer_bias_y', 0.0),
            ('magnetometer_bias_z', 0.0),
            ('load_from_file', True),
            ('save_to_file', True)
        ])

        self.x_min = 10000
        self.y_min = 10000
        self.z_min = 10000

        self.x_max = -10000
        self.y_max = -10000
        self.z_max = -10000

        self.x_mean = None
        self.y_mean = None
        self.z_mean = None

        self.last_mag = None
        self.msg = MagneticField()

        self.frame_id = 'imu'
        self.started = False

        self.measuring_time = Duration(seconds=self.get_parameter('measuring_time').value)
        self.finish_measuring_time = Time(seconds=0, clock_type=self.get_clock().clock_type)
        self.two_d_mode = bool(self.get_parameter('2d_mode').value)  # only calibrate in a plane
        # If None, the axis will be autodetected; otherwise, specify either "X", "Y" or "Z"
        self.ignore_axis = self.get_parameter('2d_mode_ignore_axis').value

        load_from_params = self.get_parameter('load_from_params').value
        load_from_params = load_from_params and self.has_parameter('magnetometer_bias_x')
        load_from_params = load_from_params and self.has_parameter('magnetometer_bias_y')
        load_from_params = load_from_params and self.has_parameter('magnetometer_bias_z')

        default_file = os.path.join(os.environ.get('HOME', ''), '.ros', 'magnetometer_calib.yaml')
        self.declare_parameter('calibration_file_path', default_file)
        self.calibration_file = self.get_parameter('calibration_file_path').value

        load_from_file = self.get_parameter('load_from_file').value
        load_from_file = load_from_file and len(self.calibration_file) > 0 and os.path.exists(self.calibration_file)

        self.save_to_file = self.get_parameter('save_to_file').value

        self.pub = self.create_publisher(
            MagneticField, 'imu/mag_bias', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.speak_pub = self.create_publisher(String, 'speak/warn', QoSProfile(depth=1))

        self.sub = None  # will be initialized in subscribe()

        self.calibrate_server = self.create_service(
            Trigger, 'calibrate_magnetometer', callback=self.calibrate_service_cb,
            qos_profile=qos_profile_services_default)

        if load_from_file:
            with open(self.calibration_file, 'r') as f:
                data = yaml.safe_load(f)
            self.x_mean = float(data.get('magnetometer_bias_x', 0.0))
            self.y_mean = float(data.get('magnetometer_bias_y', 0.0))
            self.z_mean = float(data.get('magnetometer_bias_z', 0.0))
            self.get_logger().warning(f'Magnetometer calibration loaded from file {self.calibration_file}.')
            self.pub_msg(allow_save=False)
        elif load_from_params:
            self.x_mean = float(self.get_parameter('magnetometer_bias_x').value)
            self.y_mean = float(self.get_parameter('magnetometer_bias_y').value)
            self.z_mean = float(self.get_parameter('magnetometer_bias_z').value)
            self.get_logger().warning('Magnetometer calibration loaded from parameters.')
            self.pub_msg()

    def subscribe(self):
        self.started = False
        self.sub = self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, QoSProfile(depth=100))

    def calibrate_service_cb(self, request, response):
        if self.sub or self.get_clock().now() < self.finish_measuring_time:
            self.get_logger().warning('Calibration in progress, ignoring another request')
            response.success = False
            response.message = 'Calibration already running'
            return response

        self.subscribe()
        response.success = True
        response.message = (f'Calibration started, rotate the robot for the following '
                            f'{self.measuring_time.nanoseconds / 1e9} seconds')
        return response

    def speak(self, message: str) -> None:
        msg = String()
        msg.data = message
        self.speak_pub.publish(msg)

    def mag_callback(self, msg: MagneticField) -> None:
        self.frame_id = msg.header.frame_id
        self.last_mag = msg

        if not self.started and self.get_clock().now().nanoseconds == 0:
            return

        if not self.started:
            self.started = True
            self.finish_measuring_time = self.get_clock().now() + self.measuring_time

            log = (f'Started magnetometer calibration, rotate the robot several times in the following '
                   f'{self.measuring_time.nanoseconds / 1e9} seconds.')
            self.get_logger().warning(log)
            self.speak(log)

        if self.get_clock().now() < self.finish_measuring_time:
            mag = msg.magnetic_field

            if mag.x > self.x_max:
                self.x_max = mag.x
            elif mag.x < self.x_min:
                self.x_min = mag.x

            if mag.y > self.y_max:
                self.y_max = mag.y
            elif mag.y < self.y_min:
                self.y_min = mag.y

            if mag.z > self.z_max:
                self.z_max = mag.z
            elif mag.z < self.z_min:
                self.z_min = mag.z
        else:
            self.get_logger().warning('Magnetometer calibrated')
            self.speak('Magnetometer calibrated')
            self.destroy_subscription(self.sub)  # unsubscribe the mag messages
            self.sub = None
            self.set_means()
            self.pub_msg()

    def set_means(self):
        self.x_mean = (self.x_min + self.x_max) / 2
        self.y_mean = (self.y_min + self.y_max) / 2
        self.z_mean = (self.z_min + self.z_max) / 2

        self.get_logger().warning(f'Means x: {self.x_mean} y: {self.y_mean} z: {self.z_mean}')

        if self.two_d_mode:
            x_range = self.x_max - self.x_min
            y_range = self.y_max - self.y_min
            z_range = self.z_max - self.z_min

            self.get_logger().info(f'range {x_range} {y_range} {z_range}')

            free_axis = None
            if self.ignore_axis != '':
                free_axis = self.ignore_axis.upper()
            elif x_range < min(0.75 * y_range, 0.75 * z_range):
                free_axis = 'X'
            elif y_range < min(0.75 * x_range, 0.75 * z_range):
                free_axis = 'Y'
            elif z_range < min(0.75 * x_range, 0.75 * y_range):
                free_axis = 'Z'

            if free_axis == 'X':
                self.x_mean = 0.
            elif free_axis == 'Y':
                self.y_mean = 0.
            elif free_axis == 'Z':
                self.z_mean = 0.

            if free_axis is not None:
                self.get_logger().info(
                    f'Magnetometer calibration finished in 2D mode with {free_axis} axis uncalibrated.')
            else:
                self.get_logger().warning(
                    'Magnetometer calibration in 2D mode requested, but autodetection of the free axis failed. '
                    'Did you rotate the robot? Or did you do move with the robot in all 3 axes?')

        if self.last_mag is not None:
            magnitude = np.linalg.norm([
                self.last_mag.magnetic_field.x - self.x_mean,
                self.last_mag.magnetic_field.y - self.y_mean,
                self.last_mag.magnetic_field.z - self.z_mean])
            logfn = self.get_logger().info if magnitude < 1e-4 else self.get_logger().warning
            logfn(f'Magnitude of the calibrated magnetic field is {magnitude} T.')

    def pub_msg(self, allow_save=True):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = self.frame_id

        self.msg.magnetic_field.x = self.x_mean
        self.msg.magnetic_field.y = self.y_mean
        self.msg.magnetic_field.z = self.z_mean

        self.pub.publish(self.msg)

        self.set_parameters([Parameter('magnetometer_bias_x', Parameter.Type.DOUBLE, self.x_mean)])
        self.set_parameters([Parameter('magnetometer_bias_y', Parameter.Type.DOUBLE, self.y_mean)])
        self.set_parameters([Parameter('magnetometer_bias_z', Parameter.Type.DOUBLE, self.z_mean)])

        if self.save_to_file and allow_save:
            calib_dir = os.path.dirname(self.calibration_file)
            if not os.path.exists(calib_dir):
                try:
                    os.makedirs(calib_dir)
                except OSError:
                    self.get_logger().error('Could not create folder for storing calibration ' + calib_dir)

            if os.path.exists(calib_dir):
                data = {
                    'magnetometer_bias_x': self.x_mean,
                    'magnetometer_bias_y': self.y_mean,
                    'magnetometer_bias_z': self.z_mean,
                }
                try:
                    with open(self.calibration_file, 'w') as f:
                        yaml.safe_dump(data, f)
                    self.get_logger().warning('Saved magnetometer calibration to file ' + self.calibration_file)
                except Exception as e:
                    self.get_logger().error(f'An error occured while saving magnetometer calibration to file '
                                            f'{self.calibration_file}: {e}')
            else:
                self.get_logger().error(f'Could not store magnetometer calibration to file {self.calibration_file}. '
                                        f'Cannot create the file.')


def main(args=None):
    rclpy.init(args=args)

    try:
        rclpy.spin(MagBiasObserver())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
