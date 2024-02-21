# BRUCE_SENSE

Sensory for BRUCE. This branch contains ONLY the code running on the Pi Pico(BRUCE_SENSE_PICO) which collects the IMU data and the foot contact sensor data, as well as controls the liquid cooling system.

### Dependencies

We use the Butterworth filter from the Arduino Filters(https://github.com/tttapa/Arduino-Filters) library. 

In your Arduino IDE, go to: Sketch -> Include Library -> Add .ZIP Library..., and select the Arduino-Filters.zip file in the Dependency folder.
 
### What's New
1. Increase sampling rate of sensors to 2kHz.
2. Enable two-stage Low-Pass filters on IMU.
3. Add a 2nd order Butterworth filter.
4. Move some important timing constants to the beginning of the code.

### Key Usage Points
1. Cut-off frequency of Butterworth filter should be above 10Hz and below 100Hz.
2. Preferred minimum data send frequency is 1kHz.
3. Raw data is after the two-stage Low-Pass filters on IMU.
4. Calibration is still using raw data.
5. If raw data preferred, comment line 812 send_filtered_data(); and uncomment line 811 send_data();


