/*
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "January 31, 2024"
__project__   = "BRUCE"
__version__   = "0.0.4"
__status__    = "Product"
*/

#include <Servo.h>
#include <EEPROM.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <Adafruit_NeoPixel.h>
#include "ISM330DHCX.h"

#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

// Float32 <=> 4 Bytes
typedef union {
  float floating;
  uint8_t binary[4];
} float32;


// Timing Constants
// Sampling frequency
const double f_s = 2000; // Hz
// Cut-off frequency (-3 dB)
const double f_c = 20; // Hz
// Normalized cut-off frequency
const double f_n = 2 * f_c / f_s;
// Send data frequency
const double f_send = 1000; // Hz

// Time Regulation
static const uint32_t s2us = 1000000;  // microsecond to second
uint32_t loop_start_time = 0;

// 2nd-order Butterworth filter
auto filter_ax = butter<2>(f_n); // Acceleration filter
auto filter_ay = butter<2>(f_n); // Acceleration filter
auto filter_az = butter<2>(f_n); // Acceleration filter
auto filter_ox = butter<2>(f_n); // Gyro filter
auto filter_oy = butter<2>(f_n); // Gyro filter
auto filter_oz = butter<2>(f_n); // Gyro filter


// IMU
i2c_inst_t *i2c_IMU = i2c0;

static const uint8_t SDA_PIN = 12;
static const uint8_t SCL_PIN = 13;

float omega_scale = 1;
float accel_scale = 1;
uint8_t imu_timestamp = 1;     // internal timestamp
float32 omega[3] = {0, 0, 0};  // angular rate in rad/s
float32 accel[3] = {0, 0, 0};  // linear acceleration in m/s^2
float32 omega_filtered[3] = {0, 0, 0};  // Filtered angular rate in rad/s
float32 accel_filtered[3] = {0, 0, 0};  // Filtered linear acceleration in m/s^2
float32 omega_raw[3] = {0, 0, 0};  // omega_raw = omega + omega_bias
float32 accel_raw[3] = {0, 0, 0};  // accel_raw = accel + accel_bias
float32 omega_bias[3] = {0, 0, 0};
float32 accel_bias[3] = {0, 0, 0};

static const float read_IMU_frequency = f_s;  // Hz
static const uint32_t read_IMU_dt = 1 / read_IMU_frequency * s2us;  // us
uint32_t last_read_IMU_time = 0;
uint32_t last_read_IMU_elapse = 0;


// Foot Contact
static const uint8_t FOOT_CONTACT_PIN[4] = {10, 11, 9, 8};  // right toe, right heel, left toe, left heel

bool foot_contact[4] = {false, false, false, false};
uint8_t foot_contact_data = 0b10000000;  // last 4 bits: left heel, left toe, right heel, right toe

static const float read_foot_contact_frequency = f_s;  // Hz
static const uint32_t read_foot_contact_dt = 1 / read_foot_contact_frequency * s2us;  // us
uint32_t last_read_foot_contact_time = 0;
uint32_t last_read_foot_contact_elapse = 0;


// Cooling System
static const uint8_t FAN_PIN  = 6;
static const uint8_t PUMP_PIN = 7;
Servo PUMP;

int8_t       BEAR_temp = 20;  // BEAR temperature
uint8_t     pump_speed = 30;  // 0-180
uint8_t pump_min_speed = 30;
uint8_t pump_max_speed = 80;
uint8_t      fan_speed = 99;  // 0-100
uint8_t  cooling_speed = 0;   // 0-10

static const uint32_t reset_pump_dt = 300 * 1000;  // ms (reset the pump to prevent annoying beeping)
uint32_t last_reset_pump_time = 0;


// NEO
static const uint8_t NEO_PIN = 16;
static const uint8_t NEO_NUM = 1;
Adafruit_NeoPixel pixel(NEO_NUM, NEO_PIN, NEO_GRB + NEO_KHZ800);


// Pico Mode
uint8_t IDLE_MODE           = 0;
uint8_t NOMINAL_MODE        = 1;
uint8_t RESET_MODE          = 2;
uint8_t CALIBRATION_MODE    = 3;
uint8_t INITIALIZATION_MODE = 99;
uint8_t pico_mode = INITIALIZATION_MODE;


// Pico Status
uint8_t IN_INITIALIZATION_MODE = 0;
uint8_t IN_IDLE_MODE           = 1;
uint8_t IN_NOMINAL_MODE        = 2;
uint8_t IN_CALIBRATION_MODE    = 3;
uint8_t FOOT_IN_CONTACT        = 4;
uint8_t DELAY_WARNING          = 5;
uint8_t COOLING_SYSTEM         = 6;
bool pico_status[7] = {true, false, false, false, false, false, false};

uint32_t pico_color[7] = {pixel.Color(10,  0,  0),   // green    - IN_INITIALIZATION_MODE
                          pixel.Color(10, 10, 10),   // white    - IN_IDLE_MODE
                          pixel.Color( 0,  0,  0),   // off      - IN_NOMINAL_MODE
                          pixel.Color(10,  0, 10),   // cyan     - IN_CALIBRATION_MODE
                          pixel.Color( 0, 10, 10),   // red/blue - FOOT_IN_CONTACT
                          pixel.Color(10, 10,  0),   // yellow   - DELAY_WARNING
                          pixel.Color( 2, 10,  0)};  // amber    - COOLING_SYSTEM

static const float show_pico_status_frequency = 100;  // Hz
static const uint32_t show_pico_status_dt = 1 / show_pico_status_frequency * s2us;  // us
uint32_t last_show_pico_status_time = 0;
uint32_t last_show_pico_status_elapse = 0;


// Send Data to PC
// [0xFF(1), REST_LENGTH(1), ACCEL_X(4), ACCEL_Y(4), ACCEL_Z(4), OMEGA_X(4), OMEGA_Y(4), OMEGA_Z(4), FOOT_CONTACT(1), CHECKSUM(1), 0xFE(1)]
uint8_t send_data_packet[29] = {0xFF, 27,
                                0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                                0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                                0, 228, 0xFE};

static const float send_data_frequency = f_send;  // Hz
static const uint32_t send_data_dt = 1 / send_data_frequency * s2us;  // us
uint32_t last_send_data_time = 0;
uint32_t last_send_data_elapse = 0;


// Read Data from PC
// [0xFF(1), REST_LENGTH(1), 0b00{COOLING_SPEED[4]}{PICO_MODE[2]}(1), CHECKSUM(1), 0xFE(1)]
const static uint8_t read_data_packet_length = 5;
uint8_t read_data_packet[read_data_packet_length] = {0xFF, 3,  0,  252, 0xFE};

static const float read_data_frequency = 1;  // Hz
static const uint32_t read_data_dt = 1 / read_data_frequency * s2us;  // us
uint32_t last_read_data_time = 0;
uint32_t last_read_data_elapse = 0;


// Function Declarations
int i2c_reg_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
int i2c_reg_read(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

void ping_IMU(void);
void setup_IMU(void);
void check_IMU(void);
bool read_IMU(void);
void filter_IMU(void);
bool calibrate_IMU(void);

void read_foot_contact(void);

void setup_cooling(void);
void run_cooling(uint8_t pump_speed, uint8_t fan_speed);
void idle_cooling(void);
void update_cooling(void);

void send_data(void);
void read_data(void);

void show_pico_status(void);


// Function Definitions
int i2c_reg_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t msg[len + 1];
  msg[0] = reg;
  for (int i = 0; i < len; i++)
  {
    msg[i + 1] = buf[i];
  }
  if (i2c_write_blocking(i2c, addr, msg, (len + 1), false) == len + 1)  // return number of bytes written or PICO_ERROR_GENERIC if not acknowledged
  {
    return true;
  }
  else
  {
    return false;
  }
}

int i2c_reg_read(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  if (i2c_write_blocking(i2c, addr, &reg, 1, true) == 1)
  {
    if (i2c_read_blocking(i2c, addr, buf, len, false) == len)  // return number of bytes read or PICO_ERROR_GENERIC if not acknowledged
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

void ping_IMU(void)
{
  uint8_t data[1];
  bool offline = true;
  Serial.println("Pinging IMU ...");
  while (offline)
  {
    i2c_reg_read(i2c_IMU, ISM330DHCX_ADDR, REG_WHO_AM_I, data, 1);
    if (data[0] == ISM330DHCX_ID)
    {
      offline = false;
      Serial.println("Pinging IMU Succeed.");
    }
    else
    {
      Serial.println("Pinging IMU Failed. Retrying ...");
      sleep_ms(500);
    }
  }
}

void setup_IMU(void)
{
  // block data update when reading
  uint8_t CTRL3_C = 0b01000100;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL3_C, &CTRL3_C, 1);
  
  // enable timestamp counter
  uint8_t CTRL10_C = 0b00100000;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL10_C, &CTRL10_C, 1);

  // set gyroscope for 1.66 kHz, 500dps (0b1000 01 0 0)
  // uint8_t CTRL2_G = 0b10000100;
  // set gyroscope for 3.33 kHz, 500dps (0b1001 01 0 0)
  uint8_t CTRL2_G = 0b10010100;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL2_G, &CTRL2_G, 1);
  ism330dhcx_gyro_range gyro_range = ISM330DHCX_GYRO_RANGE_500_DPS;
  switch (gyro_range) 
  {
    case ISM330DHCX_GYRO_RANGE_4000_DPS: omega_scale = 140.0; break;
    case ISM330DHCX_GYRO_RANGE_2000_DPS: omega_scale = 70.00; break;
    case ISM330DHCX_GYRO_RANGE_1000_DPS: omega_scale = 35.00; break;
    case ISM330DHCX_GYRO_RANGE_500_DPS:  omega_scale = 17.50; break;
    case ISM330DHCX_GYRO_RANGE_250_DPS:  omega_scale = 8.750; break;
    case ISM330DHCX_GYRO_RANGE_125_DPS:  omega_scale = 4.375; break;
    default: break;
  }
  omega_scale *= DEGREE_TO_RADIAN / 1000;  // raw data in milli-dps

  // set accelerometer for 1.66 kHz, 4g, data from LPF1 (0b1000 10 0 0)
  // uint8_t CTRL1_XL = 0b10001000;
  
  // set accelerometer for 3.33 kHz, 4g, data from LPF1 (0b1001 10 0 0)
  // uint8_t CTRL1_XL = 0b10011000;
  
  // set accelerometer for 3.33 kHz, 4g, data from LPF2 (0b1001 10 1 0)
  uint8_t CTRL1_XL = 0b10011010;
  
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL1_XL, &CTRL1_XL, 1);

  // Accelerometer LP2 cut-off ODR/200 (0b 101 0 0 0 0 0)
  uint8_t CTRL8_XL = 0b10100000;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL8_XL, &CTRL8_XL, 1);

  
  ism330dhcx_accel_range accel_range = ISM330DHCX_ACCEL_RANGE_4_G;
  switch (accel_range)
  {
    case ISM330DHCX_ACCEL_RANGE_16_G: accel_scale = 0.48828125000; break;
    case ISM330DHCX_ACCEL_RANGE_8_G:  accel_scale = 0.24414062500; break;
    case ISM330DHCX_ACCEL_RANGE_4_G:  accel_scale = 0.12207031250; break;
    case ISM330DHCX_ACCEL_RANGE_2_G:  accel_scale = 0.06103515625; break;
    default: break;
  }
  accel_scale *= GRAVITY_ACCELERATION / 1000;  // raw data in milli-g

  // Enables gyroscope digital LPF1 (0b 0 0 0 0 0 0 1 0)
  uint8_t CTRL4_C = 0b00000001;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL4_C, &CTRL4_C, 1);
  
  // Gyroscope low-pass filter (LPF1) bandwidth 153 (0b00000 010)
  uint8_t CTRL6_C = 0b00000010;
  i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL6_C, &CTRL6_C, 1);  
}

void check_IMU(void)
{
  uint8_t data[1];
  if (i2c_reg_read(i2c_IMU, ISM330DHCX_ADDR, REG_TIMESTAMP0, data, 1))
  {
    if (data[0] == imu_timestamp)  // IMU frozen
    {
      // software reset IMU
      uint8_t CTRL3_C = 0b01000101;
      i2c_reg_write(i2c_IMU, ISM330DHCX_ADDR, REG_CTRL3_C, &CTRL3_C, 1);
      setup_IMU();
      imu_timestamp = 0;
    }
    else
    {
      imu_timestamp = data[0];
    }
  }
}

bool read_IMU(void)
{
  uint8_t data[12];
  if (i2c_reg_read(i2c_IMU, ISM330DHCX_ADDR, REG_OUTX_L_G, data, 12))
  {
    omega_raw[0].floating = omega_scale * (int16_t)((data[1]  << 8) | data[0]);
    omega_raw[1].floating = omega_scale * (int16_t)((data[3]  << 8) | data[2]);
    omega_raw[2].floating = omega_scale * (int16_t)((data[5]  << 8) | data[4]);
    accel_raw[0].floating = accel_scale * (int16_t)((data[7]  << 8) | data[6]);
    accel_raw[1].floating = accel_scale * (int16_t)((data[9]  << 8) | data[8]);
    accel_raw[2].floating = accel_scale * (int16_t)((data[11] << 8) | data[10]);

    for (int i = 0; i < 3; i++)
    {
      omega[i].floating = omega_raw[i].floating - omega_bias[i].floating;
      accel[i].floating = accel_raw[i].floating - accel_bias[i].floating;
    }

    return true;
  }
  else
  {
    return false;
  }
}

void filter_IMU(void) 
{
  // Filter all IMU data with 2nd order Butterworth filter
  accel_filtered[0].floating = filter_ax(accel[0].floating);
  accel_filtered[1].floating = filter_ay(accel[1].floating);
  accel_filtered[2].floating = filter_az(accel[2].floating);

  omega_filtered[0].floating = filter_ox(omega[0].floating);
  omega_filtered[1].floating = filter_oy(omega[1].floating);
  omega_filtered[2].floating = filter_oz(omega[2].floating);  
}

bool calibrate_IMU(void)
{
  //send present bias to pc
  for (int i = 0; i < 3; i++)
  {
    omega[i].floating = omega_bias[i].floating;
    accel[i].floating = accel_bias[i].floating;
  }
  foot_contact_data = 0b11111111;
  send_data();

  // read data from pc
  int count = 0;
  while ((Serial.available() < read_data_packet_length) and (count < 15))  // wait for 15 seconds
  {
    sleep_ms(1000);
    count += 1;
  }
  if (Serial.available() >= read_data_packet_length)
  {
    read_data();
    if (pico_mode != CALIBRATION_MODE)
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  
  // get new bias
  float32 omega_new_bias[3] = {0, 0, 0};
  float32 accel_new_bias[3] = {0, 0, 0};
  int num = 10000;
  count = 0;
  while (count < num)
  {
    if (read_IMU())  // new data
    {
      count += 1;
      for (int i = 0; i < 3; i++)
      {
        omega_new_bias[i].floating += omega_raw[i].floating / num;
        accel_new_bias[i].floating += accel_raw[i].floating / num;
      }
    }
    sleep_ms(1);
  }
  accel_new_bias[2].floating -= GRAVITY_ACCELERATION;

  // send bias to pc
  for (int i = 0; i < 3; i++)
  {
    omega[i].floating = omega_new_bias[i].floating;
    accel[i].floating = accel_new_bias[i].floating;
  }
  send_data();

  // read data from pc
  count = 0;
  while ((Serial.available() < read_data_packet_length) and (count < 15))  // wait for 15 seconds
  {
    sleep_ms(1000);
    count += 1;
  }

  if (Serial.available() >= read_data_packet_length)
  {
    read_data();
    if (pico_mode == CALIBRATION_MODE)
    {
      for (int i = 0; i < 3; i++)
      {
        omega_bias[i].floating = omega_new_bias[i].floating;
        accel_bias[i].floating = accel_new_bias[i].floating;
        
        int idx = i * 4;
        EEPROM.write(idx,      omega_bias[i].binary[0]);
        EEPROM.write(idx + 1,  omega_bias[i].binary[1]);
        EEPROM.write(idx + 2,  omega_bias[i].binary[2]);
        EEPROM.write(idx + 3,  omega_bias[i].binary[3]);
        EEPROM.write(idx + 12, accel_bias[i].binary[0]);
        EEPROM.write(idx + 13, accel_bias[i].binary[1]);
        EEPROM.write(idx + 14, accel_bias[i].binary[2]);
        EEPROM.write(idx + 15, accel_bias[i].binary[3]);
        EEPROM.commit();
      }
      for (int i = 0; i < 3; i++)  // blink 3 times
      {
        pixel.clear();
        pixel.show();
        sleep_ms(200);
        show_pico_status();
        sleep_ms(200);
      }
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  
  return true;
}

void read_foot_contact(void)
{
  foot_contact_data = 0b10000000;
  for (int i = 0; i < 4; i++)
  {
    foot_contact[i] = !gpio_get(FOOT_CONTACT_PIN[i]);
    foot_contact_data |= foot_contact[i] << i;
  }
  
  if (foot_contact[0] | foot_contact[1] | foot_contact[2]| foot_contact[3])
  {
    pico_status[FOOT_IN_CONTACT] = true;
    pico_color[FOOT_IN_CONTACT] = pixel.Color(0, pow(10, foot_contact[0] + foot_contact[1]) - 1, pow(10, foot_contact[2] + foot_contact[3]) - 1);
  }
  else
  {
    pico_status[FOOT_IN_CONTACT] = false;
  }
}

void setup_cooling_system(void)
{
  // start pump
  PUMP.attach(PUMP_PIN, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  PUMP.writeMicroseconds(900);        // send “stop” signal to pump to initialize

  // start fan
  static uint fan_freq = 25000;
  static uint fan_wrap = 101;
  gpio_set_function(FAN_PIN, GPIO_FUNC_PWM);
  static uint fan_slice_num = pwm_gpio_to_slice_num(FAN_PIN);
  pwm_config fan_config = pwm_get_default_config();
  float fan_divider = (float)clock_get_hz(clk_sys) / (fan_freq * fan_wrap);
  pwm_config_set_clkdiv(&fan_config, fan_divider);
  pwm_config_set_wrap(&fan_config, fan_wrap);
  pwm_init(fan_slice_num, &fan_config, true);
  pwm_set_gpio_level(FAN_PIN, 0);

  sleep_ms(4000);
}

void run_cooling(uint8_t pump_goal_speed, uint8_t fan_goal_speed)
{
  PUMP.write(pump_goal_speed);
  pwm_set_gpio_level(FAN_PIN, fan_goal_speed);
  pico_status[COOLING_SYSTEM] = true;
}

void idle_cooling(void)
{
  PUMP.writeMicroseconds(900);
  pwm_set_gpio_level(FAN_PIN, 0);
  pico_status[COOLING_SYSTEM] = false;
}

void update_cooling(void)
{
  if (cooling_speed == 0) 
  {
    idle_cooling();
  }
  else
  {
    pump_speed = pump_min_speed + (pump_max_speed - pump_min_speed) * 0.1 * cooling_speed;
    run_cooling(pump_speed, fan_speed);
  }
}

void send_data(void)
{
  // [0xFF(1), REST_LENGTH(1), ACCEL_X(4), ACCEL_Y(4), ACCEL_Z(4), OMEGA_X(4), OMEGA_Y(4), OMEGA_Z(4), FOOT_CONTACT(1), CHECKSUM(1), 0xFE(1)]
  for (int i = 0; i < 3; i++)
  {
    int idx = i * 4;
    send_data_packet[idx + 2] = accel[i].binary[0];
    send_data_packet[idx + 3] = accel[i].binary[1];
    send_data_packet[idx + 4] = accel[i].binary[2];
    send_data_packet[idx + 5] = accel[i].binary[3];
  
    send_data_packet[idx + 14] = omega[i].binary[0];
    send_data_packet[idx + 15] = omega[i].binary[1];
    send_data_packet[idx + 16] = omega[i].binary[2];
    send_data_packet[idx + 17] = omega[i].binary[3];
  }
  
  send_data_packet[26] = foot_contact_data;

  send_data_packet[27] = send_data_packet[1];
  for (int i = 2; i < 27; i++)
  {
    send_data_packet[27] += send_data_packet[i];
  }
  send_data_packet[27] = ~send_data_packet[27];
  
  Serial.write(send_data_packet, 29);
}

void send_filtered_data(void)
{
  // [0xFF(1), REST_LENGTH(1), ACCEL_X(4), ACCEL_Y(4), ACCEL_Z(4), OMEGA_X(4), OMEGA_Y(4), OMEGA_Z(4), FOOT_CONTACT(1), CHECKSUM(1), 0xFE(1)]
  for (int i = 0; i < 3; i++)
  {
    int idx = i * 4;
    send_data_packet[idx + 2] = accel_filtered[i].binary[0];
    send_data_packet[idx + 3] = accel_filtered[i].binary[1];
    send_data_packet[idx + 4] = accel_filtered[i].binary[2];
    send_data_packet[idx + 5] = accel_filtered[i].binary[3];
  
    send_data_packet[idx + 14] = omega_filtered[i].binary[0];
    send_data_packet[idx + 15] = omega_filtered[i].binary[1];
    send_data_packet[idx + 16] = omega_filtered[i].binary[2];
    send_data_packet[idx + 17] = omega_filtered[i].binary[3];
  }
  
  send_data_packet[26] = foot_contact_data;

  send_data_packet[27] = send_data_packet[1];
  for (int i = 2; i < 27; i++)
  {
    send_data_packet[27] += send_data_packet[i];
  }
  send_data_packet[27] = ~send_data_packet[27];
  
  Serial.write(send_data_packet, 29);
}

void serial_send_data(void) // send data directly for debug purpose
{
  // [0xFF(1), REST_LENGTH(1), ACCEL_X(4), ACCEL_Y(4), ACCEL_Z(4), OMEGA_X(4), OMEGA_Y(4), OMEGA_Z(4), FOOT_CONTACT(1), CHECKSUM(1), 0xFE(1)]
//  Serial.print("A_X:");
//  Serial.print(accel[0].floating);
//  Serial.print(",");
//  Serial.print("A_Y:");
//  Serial.print(accel[1].floating);
//  Serial.print(",");
//  Serial.print("A_Z:");
//  Serial.println(accel[2].floating);
//  Serial.print(",");
  Serial.print("O_X:");
  Serial.print(omega_filtered[0].floating);
  Serial.print(",");
  Serial.print("O_Y:");
  Serial.print(omega_filtered[1].floating);
  Serial.print(",");
  Serial.print("O_Z:");
  Serial.println(omega_filtered[2].floating);
}

void read_data(void)
{
  while (Serial.available() >= read_data_packet_length)
  {
    if (Serial.read() == 0xFF)
    {
      if (Serial.available() >= read_data_packet_length - 1)
      {
        for (int i = 1; i < 5; i++)
        {
          read_data_packet[i] = Serial.read();
        }
        if ((read_data_packet[1] == 3) and (read_data_packet[3] == (uint8_t)(~(read_data_packet[1] + read_data_packet[2]))) and (read_data_packet[4] == 0xFE))
        {
          pico_mode     = read_data_packet[2] & 3;
          cooling_speed = read_data_packet[2] >> 2 & 15;
        }
      }
    }
  }
  while (Serial.available()) Serial.read();  // clear buffer
}


void show_pico_status(void)
{
  uint8_t idx = 0;
  if (pico_status[DELAY_WARNING])
  {
    idx = DELAY_WARNING;
  }
  else if (pico_status[FOOT_IN_CONTACT])
  {
    idx = FOOT_IN_CONTACT;
  }
  else if (pico_status[COOLING_SYSTEM])
  {
    idx = COOLING_SYSTEM;
  }
  else if (pico_status[IN_INITIALIZATION_MODE])
  {
    idx = IN_INITIALIZATION_MODE;
  }
  else if (pico_status[IN_IDLE_MODE])
  {
    idx = IN_IDLE_MODE;
  }
  else if (pico_status[IN_NOMINAL_MODE])
  {
    idx = IN_NOMINAL_MODE;
  }
  else if (pico_status[IN_CALIBRATION_MODE])
  {
    idx = IN_CALIBRATION_MODE;
  }

  pixel.clear();
  pixel.setPixelColor(0, pico_color[idx]);
  pixel.show();
}

void setup() {
  Serial.begin(1000000);
  
  // NEO
  pixel.begin();
  pixel.clear();
  pixel.show();
  show_pico_status();
  sleep_ms(1000);

  // Foot Contact
  for (int i = 0; i < 4; i++)
  {
    gpio_init(FOOT_CONTACT_PIN[i]);
    gpio_set_dir(FOOT_CONTACT_PIN[i], GPIO_IN);
    gpio_pull_up(FOOT_CONTACT_PIN[i]);
  }

  // Cooling System
  setup_cooling_system();
  run_cooling(40, 99);
  sleep_ms(1000);
  idle_cooling();

  // IMU
  EEPROM.begin(24);
  for (int i = 0; i < 3; i++)
  {
    int idx = i * 4;
    omega_bias[i].binary[0] = EEPROM.read(idx);
    omega_bias[i].binary[1] = EEPROM.read(idx + 1);
    omega_bias[i].binary[2] = EEPROM.read(idx + 2);
    omega_bias[i].binary[3] = EEPROM.read(idx + 3);

    accel_bias[i].binary[0] = EEPROM.read(idx + 12);
    accel_bias[i].binary[1] = EEPROM.read(idx + 13);
    accel_bias[i].binary[2] = EEPROM.read(idx + 14);
    accel_bias[i].binary[3] = EEPROM.read(idx + 15);
  }
  
  i2c_init(i2c0, 1000 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
  
  ping_IMU();
  setup_IMU();
  for (int i = 0; i < 3; i++)  // blink 3 times and continue since everything is good
  {
    pixel.clear();
    pixel.show();
    sleep_ms(200);
    show_pico_status();
    sleep_ms(200);
  }
  
  // Pico
  pico_mode = NOMINAL_MODE;
}

void loop() {
  for (int i = 0; i < 7; i++)
  {
    pico_status[i] = false;
  }

  if (pico_mode == NOMINAL_MODE)
  {
    pico_status[IN_NOMINAL_MODE] = true;
    while (1)
    { 
      if (time_us_32() < loop_start_time)  // time_us_32() will reset after ~70mins
      {
        last_read_IMU_time = 0;
        last_read_foot_contact_time = 0;
        last_send_data_time = 0;
        last_read_data_time = 0;
        last_show_pico_status_time = 0;
      }
      loop_start_time = time_us_32();

      pico_status[DELAY_WARNING] = false;

      // read IMU
      last_read_IMU_elapse = time_us_32() - last_read_IMU_time;
      if (last_read_IMU_elapse >= read_IMU_dt)
      {
        last_read_IMU_time = time_us_32();
        
        read_IMU();
        check_IMU();
        filter_IMU();
        
        if (last_read_IMU_elapse > read_IMU_dt * 1.5)
        {
          pico_status[DELAY_WARNING] = true;
        }
      }

      // read foot contact
      last_read_foot_contact_elapse = time_us_32() - last_read_foot_contact_time;
      if (last_read_foot_contact_elapse >= read_IMU_dt)
      {
        last_read_foot_contact_time = time_us_32();
        
        read_foot_contact();
        
        if (last_read_foot_contact_elapse > read_foot_contact_dt * 1.5)
        {
          pico_status[DELAY_WARNING] = true;
        }
      }

      // send data
      last_send_data_elapse = time_us_32() - last_send_data_time;
      if (last_send_data_elapse >= send_data_dt)
      {
        last_send_data_time = time_us_32();
        
        // send_data();
        send_filtered_data();
        // serial_send_data();
        
        if (last_send_data_elapse > send_data_dt * 1.5)
        {
          pico_status[DELAY_WARNING] = true;
        }
      }

      // read data
      last_read_data_elapse = time_us_32() - last_read_data_time;
      if (last_read_data_elapse >= read_data_dt)
      {
        last_read_data_time = time_us_32();
        
        read_data();
        update_cooling();
        if (pico_mode != NOMINAL_MODE) break;
        
        if (last_read_data_elapse > read_data_dt * 1.5)
        {
          pico_status[DELAY_WARNING] = true;
        }
      }

      // show pico status
      last_show_pico_status_elapse = time_us_32() - last_show_pico_status_time;
      if (last_show_pico_status_elapse >= show_pico_status_dt)
      {
        last_show_pico_status_time = time_us_32();
        show_pico_status();
      }
    }
  }
  else
  {
    idle_cooling();
    
    if (pico_mode == IDLE_MODE)
    {
      pico_status[IN_IDLE_MODE] = true;
      // last_reset_pump_time = millis();
      while (1)
      {
        if (millis() < last_reset_pump_time)
        {
          last_reset_pump_time = 0;
        }

        read_foot_contact();
        read_data();
        if (pico_mode != IDLE_MODE) break;
        show_pico_status();

        if (millis() - last_reset_pump_time >= reset_pump_dt)
        {
          last_reset_pump_time = millis();
          run_cooling(pump_speed, 0);
          sleep_ms(500);
          idle_cooling();
        }
        else
        {
          sleep_ms(10);
        }
      }
    }
    else if (pico_mode == CALIBRATION_MODE)
    {
      pico_status[IN_CALIBRATION_MODE] = true;
      show_pico_status();
      calibrate_IMU();
      pico_mode = IDLE_MODE;
      sleep_ms(1000);
    }
    else if (pico_mode == RESET_MODE)
    {
      setup_IMU();
      pico_mode = IDLE_MODE;
    }
  }
}
