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

import pdb
import struct
import time
import sys

from itertools import chain

import numpy as np
import serial

from bruce_sense.TIMING_TABLE import *
from bruce_sense.CONTROL_TABLE import *


class PKT(object):
    def __init__(self, port, baudrate, timeout=0.0):
        if not self.debug:
            self.ser = serial.Serial(
                port,
                baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout  # Default to 0 for non-blocking mode
            )
        else:
            pass

        if sys.version_info[0] < 3: # Python 2
            self.ord_adapt = lambda x : ord(x)
            self.sustr_adapt = lambda val: ''.join(chr(idx) for idx in val)
            self.sustr_loop_adapt = lambda idx, val: ''.join(chr(idx) for idx in val[idx:idx+4])
            pass
        else:  # Python 3
            self.ord_adapt = lambda x : x
            self.sustr_adapt = lambda val: bytearray(val)
            self.sustr_loop_adapt = lambda idx, val: bytearray(val[idx:idx+4])

    def close(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.close()

    def __del__(self):
        self.close()

    def __write_packet(self, packet):
        if not self.debug:
            self.ser.reset_input_buffer()
            self.ser.write(bytearray(packet))
        else:
            print("[DEBUG] :: __write_packet(): {}".format(packet))
            pdb.set_trace()

    def __read_packet(self):
        # TODO: Should we use checksum to verify return?
        status_packet = []
        while self.ser.inWaiting() < 3:
            pass
        status_packet.extend(self.ser.read(3))
        if status_packet:
            extlen = self.ord_adapt(status_packet[2])
            while self.ser.inWaiting() < extlen:
                pass
            status_packet.extend(self.ser.read(extlen))
            # Temporary absolute error watch:
            if len(status_packet) < (extlen+3):
                print("ser.read() returned too soon, status_packet length is only", len(status_packet))
                error_code = 0b10000001
                status_packet = [self.ord_adapt(idx) for idx in status_packet[4:-1]]
            else:
                error_code = status_packet[3]
                status_packet = [self.ord_adapt(idx) for idx in status_packet[4:-1]]
        return status_packet, error_code

    def __write_data(self, add_list, data, reg_type=None):
        if reg_type == 'cfg':
            instruction = INSTRUCTION.WRITE_CFG
        elif reg_type == 'stat':
            instruction = INSTRUCTION.WRITE_STAT

        # Convert float32 data into 4 bytes and make add_data_pair
        hex_data = []
        for d in data:
            hex = self.__float32_to_hex(d)
            hex = [hex[i:i + 2] for i in range(2, len(hex), 2)]
            hex = tuple([int(x, 16) for x in hex])
            hex_data.append(hex)
        add_data_pair = []
        for i in range(len(add_list)):
            add_data_pair.append(add_list[i])
            add_data_pair.extend(hex_data[i])
        pkt_len = len(add_data_pair)+2

        checksum = self.chksum(pkt_len, instruction, add_data_pair)

        # Generate packet
        packet = self.__packet_generator(pkt_len, instruction, add_data_pair, checksum)

        # Write packet
        self.__write_packet(packet)

    def write_status_data(self, add_list, data_list):
        """
        This command is to write data to the status registers.
        :type add_list: list
        :type data_list: list
        """
        self.__write_data(add_list, data_list, reg_type='stat')
        time.sleep(DATA_WRITE_DELAY)

    def write_config_data(self, add_list, data_list):
        """
        This command is to write data to the config registers.
        :type add_list: list
        :type data_list: list
        """
        self.__write_data(add_list, data_list, reg_type='cfg')
        time.sleep(DATA_WRITE_DELAY)

    def __read_data(self, add_list, reg_type):
        """
        :type add_list: list
        """
        if reg_type == 'cfg':
            instruction = INSTRUCTION.READ_CFG
        elif reg_type == 'stat':
            instruction = INSTRUCTION.READ_STAT

        pkt_len = len(add_list)+2

        checksum = self.chksum(pkt_len, instruction, add_list)

        packet = self.__packet_generator(pkt_len, instruction, add_list, checksum)

        self.__write_packet(packet)

        rtn_pkt_len = 4*pkt_len - 3  # Return packet length

        # Timeout prevention if communication error starts occuring
        t_bus_init = time.time()
        while True:
            if self.ser.inWaiting() > rtn_pkt_len-1:
                # Whole return packet has arrived
                break
            if time.time() - t_bus_init > TIMEOUT_MAX:
                print("[BRUCE_SENSE | WARNING] :: Status response timed out. Re-sending the same packet.")
                print(self.ser.inWaiting())
                self.ser.reset_output_buffer()
                self.__write_packet(packet)
                t_bus_init = time.time()
        # while self.ser.inWaiting() < 4:
        #     pass

        status, error_code = self.__read_packet()

        return self.__hex_to_float32(status), error_code

    def read_status_data(self, add_list):
        """
        This command is to read data from the status registers.
        """
        return self.__read_data(add_list, reg_type='stat')

    def read_config_data(self, add_list):
        """
        This command is to read data from the config registers.
        """
        return self.__read_data(add_list, reg_type='cfg')

    def _ping(self):
        """
        Function used to detect a motor. Will return firmware and hardware version info.
        Return None if pinging unsuccessful.
        """
        # Type of instruction
        instruction = INSTRUCTION.PING

        # Number of parameters
        pkt_len = 2

        # Create checksum
        checksum = self.chksum(pkt_len, instruction, (0,))

        # Write packet
        self.__write_packet((0xFF, 0xFF, pkt_len, instruction, checksum))
        start_time = time.time()
        while self.ser.inWaiting() < 1:
            if time.time()- start_time > PING_TIMEOUT:
                # Timeout
                return None
            else:
                pass

        # Read status
        ping_status, error_code = self.__read_packet()

        return ping_status, error_code

    def _read_dump(self):
        """
        This command is to read data from dumper
        :return:
        """
        # Timeout prevention if communication error starts occuring
        incoming_bytes = [0, 0]
        got_packet = False
        t_bus_init = time.time()
        self.ser.reset_input_buffer()
        while True:
            if self.ser.inWaiting() > 1:
                # Check for start of packet
                incoming_bytes.extend(self.ser.read(1))
                incoming_bytes.pop(0)
                if ((incoming_bytes[1]<<8)|incoming_bytes[0]) == 0xFFFF:
                    # This is the beginning of the packet
                    got_packet = True
                    incoming_bytes.extend(self.ser.read(1))
                    break
            if time.time() - t_bus_init > TIMEOUT_MAX:
                print("[BRUCE_SENSE | WARNING] :: Status response timed out. Is dumper alright?")
                t_bus_init = time.time()
        if got_packet:
            t_bus_init = time.time()
            pktlen = incoming_bytes[2]
            while self.ser.inWaiting() < pktlen:
                if time.time() - t_bus_init > TIMEOUT_MAX:
                    print("[BRUCE_SENSE | WARNING] :: Status response timed out waiting for rest of packet")
            status_packet = self.ser.read(pktlen)
        # Temporary absolute error watch:
        if len(status_packet) < pktlen:
            print("ser.read() returned too soon, status_packet length is only", len(status_packet))
            error_code = 0b00000001
        elif status_packet[-1] is not 0x0F:
            print("Corrupted packet.")
            error_code = 0b00000010
        else:
            error_code = status_packet[0]
            contact = status_packet[-2]
        status_packet = [self.ord_adapt(idx) for idx in status_packet[1:-2]]
        return self.__hex_to_float32(status_packet), contact, error_code

    def _read_dump_YOLO(self):
        """
        This command is to read data from dumper, but it only read once and return None if no data in buffer
        :return:
        """
        # Timeout prevention if communication error starts occuring
        incoming_bytes = [0, 0]
        got_packet = False
        t_bus_init = time.time()
        self.ser.reset_input_buffer()
        while True:
            if self.ser.inWaiting() > 1:
                # Check for start of packet
                incoming_bytes.extend(self.ser.read(1))
                incoming_bytes.pop(0)
                if ((incoming_bytes[1]<<8)|incoming_bytes[0]) == 0xFFFF:
                    # This is the beginning of the packet
                    got_packet = True
                    incoming_bytes.extend(self.ser.read(1))
                    break
            if time.time() - t_bus_init > TIMEOUT_MAX:
                # Timeout with no data
                return None
        if got_packet:
            t_bus_init = time.time()
            pktlen = incoming_bytes[2]
            while self.ser.inWaiting() < pktlen:
                if time.time() - t_bus_init > TIMEOUT_MAX:
                    print("[BRUCE_SENSE | WARNING] :: Status response timed out waiting for rest of packet")
            status_packet = self.ser.read(pktlen)
        # Temporary absolute error watch:
        if len(status_packet) < pktlen:
            print("ser.read() returned too soon, status_packet length is only", len(status_packet))
            error_code = 0b00000001
        elif status_packet[-1] is not 0x0F:
            print("Corrupted packet.")
            error_code = 0b00000010
        else:
            error_code = status_packet[0]
            contact = status_packet[-2]
        status_packet = [self.ord_adapt(idx) for idx in status_packet[1:-2]]
        return self.__hex_to_float32(status_packet), contact, error_code

    # ===== Utility functions
    def __packet_generator(self, length, instruction, add_data_pair, checksum):
        """
        :type add_data_pair: list
        """
        l = (0xFF, 0xFF, length, instruction, add_data_pair, checksum)
        return tuple((chain(*(i if isinstance(i, list) else (i,) for i in l))))
        # return (0xFF, 0xFF, m_id, length, instruction, param_n, data, checksum)

    def __float32_to_hex(self, val):
        retval = hex(struct.unpack('<I', struct.pack('>f', val))[0])
        if retval[-1] == 'L':
            retval = retval[:-1]
        if len(retval) < 10:
            length = 10-len(retval)
            retval = '0x'+length*'0'+retval[2:]
        return retval

    def __hex_to_float32(self, val):
        if len(val) > 4:
            tmpval = []
            for idx in range(0, len(val), 4):
                tmpval.append(struct.unpack('<f', self.sustr_loop_adapt(idx, val))[0])
        else:
            tmpval = struct.unpack('<f', self.sustr_adapt(val))[0]
        return tmpval

    def chksum(self, length, instruction, param_n):
        return 255 - ((length + instruction + sum(param_n)) % 256)

    def _echo(self, add_list):
        """
        :type add_list: list
        """
        instruction = INSTRUCTION.READ_STAT

        pkt_len = len(add_list) + 2

        checksum = self.chksum(pkt_len, instruction, add_list)

        packet = self.__packet_generator(pkt_len, instruction, add_list, None, checksum)

        self.__write_packet(packet)

        rtn_pkt_len = 4 * pkt_len - 3  # Return packet length

        # Timeout prevention if communication error starts occuring
        rtn = []
        t_bus_init = time.time()
        while True:
            if time.time() - t_bus_init > TIMEOUT_MAX:
                print("Timeout.")
                break
        if self.ser.in_waiting > 0:
            rtn = list(self.ser.read(50))
            print("rtn: {}".format(rtn))
        return rtn
