__author__ = "X. Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2021 Westwood Robotics"
__date__ = "Aug. 01, 2022"

__version__ = "0.0.1"
__status__ = "Prototype"

# Use this to calibrate IMU when using Pico in dumper mode

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# ---------  IMPORTANT  --------------------------------
# Set all bias in BRUCE_SENSE_PICO to 0 before calibration
# Refer to setup()
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

import time

from bruce_sense import Manager
s = Manager.SENSOR(port='COM12', baudrate=2000000)


rxs = []
rys = []
rzs = []
ddxs = []
ddys = []
ddzs = []



data = {}
sample_rate = 2000

for i in range(sample_rate):
    data = s.get_dump()[0]
    # Raw acceleration:
    ddxs.append(data[0])
    ddys.append(data[1])
    ddzs.append(data[2])
    # Raw omega:
    rxs.append(data[3])
    rys.append(data[4])
    rzs.append(data[5])
    time.sleep(0.01)
    print(i/2000)

ddx_bias = sum(ddxs)/sample_rate
ddy_bias = sum(ddys)/sample_rate
ddz_bias = sum(ddzs)/sample_rate


rx_bias = sum(rxs)/sample_rate
ry_bias = sum(rys)/sample_rate
rz_bias = sum(rzs)/sample_rate

print("Calibration result:")
print("Acceleration bias: x: %f, y: %f, z: %f" % (ddx_bias, ddy_bias, ddz_bias))
print("Gyro bias: x: %f, y: %f, z: %f" % (rx_bias, ry_bias, rz_bias))