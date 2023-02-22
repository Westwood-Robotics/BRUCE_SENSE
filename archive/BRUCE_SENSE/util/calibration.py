# Set bias to zero then take data for sometime and get avrg


import time
import numpy as np

from bruce_sense import Manager
s = Manager.SENSOR(port='COM6', baudrate=2000000)
dt = 10  # take data for 2 seconds
GRAV = 9.81
# Reset bias
s.set_config(('acceleration_bias_x', 0), ('acceleration_bias_y', 0), ('acceleration_bias_z', 0),
             ('gyro_bias_x', 0), ('gyro_bias_y', 0), ('gyro_bias_z', 0))
# Read to check
a = s.get_config('acceleration_bias_x', 'acceleration_bias_y', 'acceleration_bias_z',
                  'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z')[0]
check = sum(a)
if check == 0:
    print("Bias cleared.")
else:
    print("Bias clear failed. Result: ")
    print(a)
    exit()

# Calibrate
print("Calibrating...")
start_time = time.time()
data = np.array(s.get_imu_raw()[0])
n = 1
while True:
    data += np.array(s.get_imu_raw()[0])
    n += 1
    if time.time()-start_time > dt:
        break

data = data/n
data[2] = data[2]-GRAV

print("Calibration result:")
print("Acceleration: ", data[0:3])
print("Gyro: ", data[3:])
