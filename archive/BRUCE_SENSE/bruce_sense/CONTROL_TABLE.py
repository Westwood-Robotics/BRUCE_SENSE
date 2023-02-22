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

__author__ = "X."
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2021 Westwood Robotics"
__date__ = "Apr. 07, 2021"

__version__ = "0.0.1"
__status__ = "Prototype"

"""
Control Table
"""


class INSTRUCTION:
    """
    Instruction Constants
    ---------------------
    This is the field that defines the purpose of the Packet.
    """

    PING = 0x01
    READ_STAT = 0x02
    READ_CFG = 0x03
    WRITE_STAT = 0x04
    WRITE_CFG = 0x05
    # SAVE_CFG = 0x06
    # BULK_COMM = 0x12


class CFG_REG:
    """Configuration Registers"""
    TAU = 0x01  # CF percentage
    A_BIAS_X = 0x02  # acceleration bias in m/s
    A_BIAS_Y = 0x03
    A_BIAS_Z = 0x04
    GYRO_BIAS_X = 0x05  # gyro bias in rad/s
    GYRO_BIAS_Y = 0x06
    GYRO_BIAS_Z = 0x07
    CRIT_1 = 0x08  # criterion_1: (rad/s) large drift or incorrect measurement of gyro, use a_angle for gyro_angle
    CRIT_2 = 0x09  # criterion_2: (rad/s) very slow movement, trust merely on acceleration angle to avoid drift


CFG_REG_DIC = {'tau': CFG_REG.TAU,
               'acceleration_bias_x': CFG_REG.A_BIAS_X,
               'acceleration_bias_y': CFG_REG.A_BIAS_Y,
               'acceleration_bias_z': CFG_REG.A_BIAS_Z,
               'gyro_bias_x': CFG_REG.GYRO_BIAS_X,
               'gyro_bias_y': CFG_REG.GYRO_BIAS_Y,
               'gyro_bias_z': CFG_REG.GYRO_BIAS_Z,
               'criterion_1': CFG_REG.CRIT_1,
               'criterion_2': CFG_REG.CRIT_2}


class STAT_REG:
    """Status Registers"""
    TEMP = 0x00  # Temperature
    ACCEL_X = 0x01  # compensated raw acceleration
    ACCEL_Y = 0x02
    ACCEL_Z = 0x03
    OMEGA_X = 0x04  # compensated raw omega
    OMEGA_Y = 0x05
    OMEGA_Z = 0x06
    ACCEL_NO_G_X = 0x07  # No-G acceleration
    ACCEL_NO_G_Y = 0x08
    ACCEL_NO_G_Z = 0x09
    ANGLE_X = 0x0A  # Filtered angle
    ANGLE_Y = 0x0B
    ANGLE_Z = 0x0C


STAT_REG_DIC = {'temperature': STAT_REG.TEMP,
                'acceleration_x': STAT_REG.ACCEL_X,
                'acceleration_y': STAT_REG.ACCEL_Y,
                'acceleration_z': STAT_REG.ACCEL_Z,
                'omega_x': STAT_REG.OMEGA_X,
                'omega_y': STAT_REG.OMEGA_Y,
                'omega_z': STAT_REG.OMEGA_Z,
                'no_g_acceleration_x': STAT_REG.ACCEL_NO_G_X,
                'no_g_acceleration_y': STAT_REG.ACCEL_NO_G_Y,
                'no_g_acceleration_z': STAT_REG.ACCEL_NO_G_Z,
                'angle_x': STAT_REG.ANGLE_X,
                'angle_y': STAT_REG.ANGLE_Y,
                'angle_z': STAT_REG.ANGLE_Z}
