#!usr/bin/env python

################################################################################
# Copyright 2021 Westwood Robotics Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2021 Westwood Robotics"
__date__ = "Feb. 06, 2022"

__version__ = "0.0.1"
__status__ = "Prototype"

import os

import pdb

import serial

import time

from bruce_sense import Packet

from bruce_sense.CONTROL_TABLE import *


class SENSOR(Packet.PKT):
    """
    Provides control of Dynamixel servos using PySerial
    """
    def __init__(self,
                 port = 'COM8',
                 baudrate = '115200',
                 bytesize = serial.EIGHTBITS,
                 parity = serial.PARITY_NONE,
                 stopbits = serial.STOPBITS_ONE,
                 timeout = 0.0,
                 debug = False):
        """
        Constructor for opening up the serial port.
        :param port: Port address; Should be specified per object if using multiple chains
        :param baudrate: Specified baudrate
        """

        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.connected = False

        self.ascii_art = True

        self.debug = debug # Option for ease of debugging. Set to false for proper operation.

        super(SENSOR, self).__init__(self.port, self.baudrate)

        self.welcome_msg()

    def welcome_msg(self):
        if self.ascii_art:
            print("====== BRUCE_SENSE version 0.0.1 -- Last Updated 2022.02.06 =======")
            print("==============================================================")

    # =================================================================================================================
    # ===== Status Registers
    def ping(self):
        """
        Function used to detect device. Will return firmware and hardware version info.
        Return None if pinging unsuccessful.
        """
        return self._ping()

    def get_status(self, *argv):
        """
        Get the specified data.
        :param argv: name data to read
        :return:
        """
        add_list = [STAT_REG_DIC[i] for i in argv]

        return self.read_status_data(add_list)

    def get_config(self, *argv):
        """
        Get the specified data.
        :param argv: name data to read
        :return:
        """
        add_list = [CFG_REG_DIC[i] for i in argv]

        return self.read_config_data(add_list)

    def set_config(self, *add_data_pair):
        """
        Write to specific config
        :param add_data_pair: address-data pair
        :return:
        """
        add_list = [CFG_REG_DIC[i[0]] for i in add_data_pair]
        data_list = [i[1] for i in add_data_pair]

        self.write_config_data(add_list, data_list)

    def get_imu_all(self):
        """
        Get all IMU data.
        :return:
        """
        add_list = [STAT_REG.ACCEL_X, STAT_REG.ACCEL_Y, STAT_REG.ACCEL_Z,
                    STAT_REG.OMEGA_X, STAT_REG.OMEGA_Y, STAT_REG.OMEGA_Z,
                    STAT_REG.ACCEL_NO_G_X, STAT_REG.ACCEL_NO_G_Y, STAT_REG.ACCEL_NO_G_Z,
                    STAT_REG.ANGLE_X, STAT_REG.ANGLE_Y, STAT_REG.ANGLE_Z]

        return self.read_status_data(add_list)

    def get_imu_raw(self):
        """
        Get all IMU data.
        :return:
        """
        add_list = [STAT_REG.ACCEL_X, STAT_REG.ACCEL_Y, STAT_REG.ACCEL_Z,
                    STAT_REG.OMEGA_X, STAT_REG.OMEGA_Y, STAT_REG.OMEGA_Z]

        return self.read_status_data(add_list)

    def get_temperature(self):
        """
        Get temperature data
        :return:
        """

        return self.read_status_data([STAT_REG.TEMP])

    # ======================
    # Debug functions

    def echo(self, *argv):
        add_list = [i for i in argv]
        return self._echo(add_list)

    # ======================
    # Use with only IMU_dumper
    def get_dump(self):
        """
        Get all IMU data from dumper
        :return:
        """
        return self._read_dump()



