# BRUCE_SENSE

Sensory for BRUCE. This repo contains both the code running on the Pi Pico(BRUCE_SENSE_PICO) which collects the IMU data and the foot contact sensor data (as well as liquid cooling control in the future), and the code running on the on-board computer(BRUCE_SENSE) which controls the motion of BRUCE.

### BRUCE_SENSE

To use BRUCE_SENSE, please first run setup.py to install the module. 

To read data, first import `Manager` then create an instance of `Manger.SENSOR`.

The library is still under construction and for simplicity, please only use `SENSOR.get_dump()` function for now.

### BRUCE_SENSE_PICO

Refer to _README_ in BRUCE_SENSE_PICO for more details on how to set up the pi pico for BRUCE.

