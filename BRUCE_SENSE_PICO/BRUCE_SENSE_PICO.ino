/*
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.io"
__copyright__ = "Copyright 2021 Westwood Robotics"
__date__ = "Nov. 04, 2021"

__version__ = "0.0.3"
__status__ = "Prototype" 
*/

/* Debug Note
 *  - Checked output using arduino serial plotter, confirmed that the noise still exists --- This is probably not a problem of serial communication
 *  - Tried different sensitivity settings for acceleration, confirmed that the noise still exists --- This is not a problem of sensitivity settings, but note that the higher the sensitivity, the higher the impulse magnitude
 *  - Tried SPI instead of I2C to debug communication between PICO and IMU, confirmed that the noise still exists --- This is not a problem of communication interface- Tried different sensitivity settings for acceleration, confirmed that the noise still exists --- This is not a problem of sensitivity settings, but note that the higher the sensitivity, the higher the impulse magnitude
 *  - Try different ODR, confirmed that the noise still exists --- This is not a problem of ODR
 *  - Try LPF2, confirmed that the noise still exists --- This is not a problem of LPF, but left LPF2 enabled.
 *  
 *  Conclusion: it's a bug of the chip. Looking for workarounds...
 *  - Tried moving average, condition significantly improved with a queue of length-10
 */

 
#define FW_Ver 0 // This firmware version
#define HW_Ver 0 // current hardware version
#define debug false // Set to True to output status to serial port

#include <math.h>
#include "dependency/ISM330DHCX.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#define REG_CTRL8_XL  0x17
#define Q_LEN 10 // Moving average queue length
float Q_X[Q_LEN+1] = {0}; // Global queue space for a_x, last element stores queue average
float Q_Y[Q_LEN+1] = {0}; // Global queue space for a_y, last element stores queue average
float Q_Z[Q_LEN+1] = {0}; // Global queue space for a_z, last element stores queue average

/* Serial Communication
Instructions
  PING       0x01
  READ_STAT  0x02
  READ_CFG   0x03
  WRITE_STAT 0x04
  WRITE_CFG  0x05
  
IMU Control Table
==STAT=====================================
  NA      0x00
  a_x     0x01 // compensated acceleration, m/s
  a_y     0x02 
  a_z     0x03 
  omega_x 0x04 // compensated omega, rad/s
  omega_y 0x05 
  omega_z 0x06 
  a_nog_x 0x07 // No-G acceleration, m/s
  a_nog_y 0x08 
  a_nog_z 0x09 
  angle_x 0x0A // Filtered angle, rad
  angle_y 0x0B 
  angle_z 0x0C 
==CFG=====================================
  dt          0x00  // loop time
  tau         0x01  // CF percentage 
  a_bias_x    0x02  // acceleration bias in m/s
  a_bias_y    0x03
  a_bias_z    0x04
  gyro_bias_x 0x05  // gyro bias in rad/s
  gyro_bias_y 0x06
  gyro_bias_z 0x07  
  crit_1      0x08  // criterion_1: (rad/s)large drift or incorrect measurement of gyro, use a_angle for gyro_angle
  crit_2      0x09  // criterion_2: (rad/s)movement is very slow, trust merely on acceleration angle to avoid drift  */

typedef union {
 float floatingPoint;
 uint8_t binary[4];
} float32;  // content of binary[0] is the lowest LSByte

typedef union {
 short shortInt;
 uint8_t binary[2];
} int16;  // content of binary[0] is the lowest LSByte

float32 STAT[13]; // STAT date space
float32 CFG[10]; // CFG date space

// Pointers for easier programming
// ==STAT=========================
float32 *temperature = &STAT[0]; 
float32 *angle = &STAT[10]; // filtered angle
float32 *angle_x = &STAT[10]; // filtered angle
float32 *angle_y = &STAT[11];
float32 *angle_z = &STAT[12];
// ==CFG==========================
float32 *dt = &CFG[0];  // loop time in us
float32 *tau = &CFG[1];  // CF percentage
float32 *a_bias_x = &CFG[2];  // acceleration bias
float32 *a_bias_y = &CFG[3];
float32 *a_bias_z = &CFG[4]; 
float32 *gyro_bias_x = &CFG[5];  // gyro bias
float32 *gyro_bias_y = &CFG[6]; 
float32 *gyro_bias_z = &CFG[7]; 
float32 *criterion_1 = &CFG[8];  // CF criteria
float32 *criterion_2 = &CFG[9];

uint8_t error = 0b10000000; // Error LSB: overtime, xx,xx,xx...
uint8_t contact = 0b10000000; // Contact status, MSB always 1 and the last 4 bits from MSB: left foot toe heel, right foot toe heal


// Pinout
static const uint LED_PIN = 25;

static const uint SDA_PIN = 16;
static const uint SCL_PIN = 17;

static const uint LTOE_PIN = 18;
static const uint LHEEL_PIN = 19;
static const uint RTOE_PIN = 20;
static const uint RHEEL_PIN = 21;

// Ports
i2c_inst_t *i2c = i2c0;

// Other constants
// static const float PI = 3.14159265359;
static const float us2s = 0.000001; // microSecond to second
#define dt_default  1000   // default dt
static const float tau_default = 0.05; // default tau


// Global variables
unsigned long t_final = 0; // loop end time in microseconds
unsigned long t_reset = 0; // loop end time in miliseconds
uint8_t packetLen = 0;
uint16_t incomingBytes = 0;
float gyro_scale = 1; // range is in milli-dps per bit!
float accel_scale = 1; // range is in milli-g per bit!

/*******************************************************************************
 * Function Declarations
 */

int reg_write(i2c_inst_t *i2c, 
              const uint addr, 
              const uint8_t reg, 
              uint8_t *buf,
              const uint8_t nbytes);

int reg_read(i2c_inst_t *i2c,
             const uint addr,
             const uint8_t reg,
             uint8_t *buf,
             const uint8_t nbytes);

void readIMU();

void readContact();

void dump();

float moving_average(float *q, 
                     float new_data);

/*******************************************************************************
 * Function Definitions
 */

// Write 1 byte to the specified register
int reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive registers.
int reg_read( i2c_inst_t *i2c,
              const uint addr,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

// Calculate moving average
float moving_average(float *q, 
                     float new_data){
  q[Q_LEN]-=q[0]/Q_LEN; // Remove the portion of the first value in the queue from average
  for(int i=0; i<(Q_LEN-1); i++){
    q[i]=q[i+1];    
  } // Shift queue forward 
  q[Q_LEN-1] = new_data; // Add new data into queue
  q[Q_LEN]+= q[Q_LEN-1]/Q_LEN; // Add the portion of the last value in the queue into average
  return q[Q_LEN];
}

// Read and filter data from IMU, then populate STAT date space
void readIMU(){
    // Buffer to store raw reads
    uint8_t data[12];
    // Declare data processing buffer
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
    float gyro_x_f;
    float gyro_y_f;
    float gyro_z_f;
    float cp;
    float sp;
    float cr;
    float sr;
    float tp;
    float a_angle[] = {0,0,0}; // acceleration angle
    float gyro_angle[3] = {0,0,0}; // gyro angle intigrated from omega over time

    // Read X, Y, and Z acc and gyro values from registers (16 bits each, 12 bytes in total)
    reg_read(i2c, ISM330DHCX_ADDR, REG_X_L_G, data, 12);
    // Convert 2 bytes (little-endian) into 16-bit integer (signed)
    gyro_x = (int16_t)((data[1] << 8) | data[0]);
    gyro_y = (int16_t)((data[3] << 8) | data[2]);
    gyro_z = (int16_t)((data[5] << 8) | data[4]);
    acc_x = (int16_t)((data[7] << 8) | data[6]);
    acc_y = (int16_t)((data[9] << 8) | data[8]);
    acc_z = (int16_t)((data[11] << 8) | data[10]);
    // Convert measurements to [m/s^2] and [rad/s]
    gyro_x_f = gyro_x * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
    gyro_y_f = gyro_y * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
    gyro_z_f = gyro_z * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
    acc_x_f = acc_x * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
    acc_y_f = acc_y * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
    acc_z_f = acc_z * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;

    float a[] = {acc_x_f + (*a_bias_x).floatingPoint, 
                 acc_y_f + (*a_bias_y).floatingPoint,
                 acc_z_f + (*a_bias_z).floatingPoint}; 
    float omega[] = {gyro_x_f + (*gyro_bias_x).floatingPoint,
                     gyro_y_f + (*gyro_bias_y).floatingPoint,
                     gyro_z_f + (*gyro_bias_z).floatingPoint};  
    float grav_vec[3]; // Gravity vector

    // Filter acceleration data with moving average
    a[0] = moving_average(Q_X, a[0]);
    a[1] = moving_average(Q_Y, a[1]);
    a[2] = moving_average(Q_Z, a[2]);

    // angle calculation 
    a_angle[0] = atan2(a[1], a[2]);
    a_angle[1] = -atan2(a[0], sqrt(a[1]*a[1] + a[2]*a[2]));
    
    cr = cos((*angle_x).floatingPoint);
    sr = sin((*angle_x).floatingPoint);
    cp = cos((*angle_y).floatingPoint);
    sp = sin((*angle_y).floatingPoint);
    if (abs(cp) < 0.0001){
      cp = 0.0001;
    }
    tp = sp / cp;   

    gyro_angle[0] = (*angle_x).floatingPoint + (omega[0] + omega[1]*tp*sr + omega[2]*tp*cr)*(*dt).floatingPoint*us2s;
    gyro_angle[1] = (*angle_y).floatingPoint + (omega[1]*cr - omega[2]*sr)*(*dt).floatingPoint*us2s;
    gyro_angle[2] = (*angle_z).floatingPoint + (omega[1]*sr/cp + omega[2]*cr/cp)*(*dt).floatingPoint*us2s;
    
    // gyro_angle[0] = (*angle_x).floatingPoint + omega[0]*(*dt).floatingPoint*us2s;
    // gyro_angle[1] = (*angle_y).floatingPoint + omega[1]*(*dt).floatingPoint*us2s;
    // Yaw
    // gyro_angle[2] = (*angle_z).floatingPoint + omega[2]*(*dt).floatingPoint*us2s;


    // Main CF
    //  for (int i = 0; i<2; i++){
    ////    (*(angle+i)).floatingPoint = (1 - (*tau).floatingPoint)*gyro_angle[i] + (*tau).floatingPoint*a_angle[i];
    //      (*(angle+i)).floatingPoint = a_angle[i];
    //  }
    //  
    float gsum = abs(omega[0])+abs(omega[1])+abs(omega[2]);
    for (int i = 0; i<2; i++){
    //    if (abs(gyro_angle[i] - a_angle[i]) > (*criterion_1).floatingPoint){
    //      // Correct for very large drift or incorrect measurement of gyro by longer loop time
    //      gyro_angle[i] = a_angle[i];
    //    }
        if (gsum > (*criterion_2).floatingPoint){
        (*(angle+i)).floatingPoint = (1 - (*tau).floatingPoint)*gyro_angle[i] + (*tau).floatingPoint*a_angle[i];
        }
        else{
        // if movement is very slow, trust merely on acceleration angle to avoid drift
        (*(angle+i)).floatingPoint = a_angle[i];
        }
    }
    (*angle_z).floatingPoint = gyro_angle[2];

    // Calculate no G acceleration
    grav_vec[0] = -sin((*angle_y).floatingPoint)*GRAVITY_EARTH;
    grav_vec[1] =  cos((*angle_y).floatingPoint)*sin((*angle_x).floatingPoint)*GRAVITY_EARTH;
    grav_vec[2] =  cos((*angle_y).floatingPoint)*cos((*angle_x).floatingPoint)*GRAVITY_EARTH;

    // Write data to STAT space
    for (int i = 0; i<3; i++){
        // compensated raw acceleration
        STAT[1+i].floatingPoint = a[i];
        // compensated raw omega
        STAT[4+i].floatingPoint = omega[i];
        // no G acceleration
        if (a[2]>0){
        STAT[7+i].floatingPoint = a[i]-grav_vec[i];
        }
        else{
        STAT[7+i].floatingPoint = a[i]+grav_vec[i];
        }
    }


}

// Get Foot Contact Status and update 'contact'
void readContact(){
  // contact = 0b10000000: Contact status, MSB always 1 and the last 4 bits: left foot toe heel, right foot toe heal

  contact = (gpio_get_all() & 0b1111000000000000000000)>>18; 
  
  
  //contact = contact | gpio_get(LTOE_PIN)<<3 
}

// Dump the data to serial port
void dump(){  
    /* Packet: [0xFF 0xFF packetLen_outgoing Error 
                ACCEL_X ACCEL_Y ACCEL_Z OMEGA_X OMEGA_Y OMEGA_Z
                ACCEL_NO_G_X ACCEL_NO_G_Y ACCEL_NO_G_Z 
                ANGLE_X ANGLE_Y ANGLE_Z 0 0]*/
    uint8_t rtnPkt[54] = {};
    rtnPkt[0] = 0xFF;
    rtnPkt[1] = 0xFF;
    rtnPkt[2] = 51;
    rtnPkt[3] = error;
    rtnPkt[52] = contact;
    rtnPkt[53] = 0x0F;
    for (int i=1; i<13; i++){
        // build packet
         rtnPkt[i*4]   = STAT[i].binary[0];
         rtnPkt[i*4+1] = STAT[i].binary[1];
         rtnPkt[i*4+2] = STAT[i].binary[2];
         rtnPkt[i*4+3] = STAT[i].binary[3];  
    }

    Serial.write(rtnPkt,54);
}


// Device setup
void setup() {
    // Buffer to store raw reads
    uint8_t data[1];

    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize Foot Contact pins
    gpio_init(LTOE_PIN);
    gpio_pull_up(LTOE_PIN);
    gpio_set_dir(LTOE_PIN, GPIO_IN);

    gpio_init(LHEEL_PIN);
    gpio_pull_up(LHEEL_PIN);
    gpio_set_dir(LHEEL_PIN, GPIO_IN);
    
    gpio_init(RTOE_PIN);
    gpio_pull_up(RTOE_PIN);
    gpio_set_dir(RTOE_PIN, GPIO_IN);

    gpio_init(RHEEL_PIN);
    gpio_pull_up(RHEEL_PIN);
    gpio_set_dir(RHEEL_PIN, GPIO_IN);

    // Initialize chosen serial port
    Serial.begin(115200);

    //Initialize I2C port at 1000 kHz
    i2c_init(i2c, 1000 * 1000);

    // Initialize I2C pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // CFG data initiation
    (*dt).floatingPoint = dt_default;
    (*tau).floatingPoint = tau_default;
    (*a_bias_x).floatingPoint = 0.05616613;//0.26404677;
    (*a_bias_y).floatingPoint = 0.19473464;//-0.12604313;
    (*a_bias_z).floatingPoint = -0.04458105;//0.0390042;
    (*gyro_bias_x).floatingPoint = -0.00721309;//-0.00658039;
    (*gyro_bias_y).floatingPoint = 0.01036009;//0.0093528;
    (*gyro_bias_z).floatingPoint = 0.00560769;//0.00566089;
    (*criterion_1).floatingPoint = 2;
    (*criterion_2).floatingPoint = 0.005;

    // turn on LED and wait for 0.5s
    gpio_put(LED_PIN, true);
    sleep_ms(500);

    // Read device ID to make sure that we can communicate with the ISM330DHCX
    reg_read(i2c, ISM330DHCX_ADDR, REG_DEVID, data, 1);
    if (data[0] != ISM330DHCX_ID) {
        printf("ERROR: Could not communicate with ISM330DHCX\r\n");
        // turn off LED and wait for 0.5s
        gpio_put(LED_PIN, false);
        sleep_ms(500);
        while (true); // Stall device since critical error
    }
    else{
        //LED quick blink 5 times and stay on
        for(int i=0; i<5; i++){
            gpio_put(LED_PIN, false);
            sleep_ms(100);
            gpio_put(LED_PIN, true);
            sleep_ms(100);
        }
        printf("Found ISM330DHCX!\r\n");

        // Set Accelerometer Control register for 3.33 kHz, 2g, data from LPF1 (0b1001 10 0 0)
        uint8_t CTRL1_XL = 0b10101000;
        reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL1_XL, &CTRL1_XL, 1);
        // Set Accelerometer Control register for LPF2, no HP filter, HPCF_XL = 000 (0b000 0 0 0 0 0)        
        uint8_t CTRL8_XL = 0b00000000;        
        reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL8_XL, &CTRL8_XL, 1);
        
        // Set Gyro Control register for 3.33 kHz, 500DPS (0b1001 01 0 0)
        uint8_t CTRL2_G = 0b10010100;
        reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL2_G, &CTRL2_G, 1);
        printf("CTRL1_XL Set to 0x%02x; CTRL2_G Set to %02x.\r\n", CTRL1_XL, CTRL2_G);
        sleep_ms(200);        

        // Decide scale factors from range selections
        ism330dhcx_gyro_range_t gyro_range = ISM330DHCX_GYRO_RANGE_500_DPS;
        switch (gyro_range)
        {
        case ISM330DHCX_GYRO_RANGE_4000_DPS: gyro_scale = 140.0;
            break;
        case ISM330DHCX_GYRO_RANGE_2000_DPS: gyro_scale = 70.0;
            break;
        case ISM330DHCX_GYRO_RANGE_1000_DPS: gyro_scale = 35.0;
            break;
        case ISM330DHCX_GYRO_RANGE_500_DPS: gyro_scale = 17.50;
            break;
        case ISM330DHCX_GYRO_RANGE_250_DPS: gyro_scale = 8.75;
            break;
        case ISM330DHCX_GYRO_RANGE_125_DPS: gyro_scale = 4.375;
            break;        
        default:
            break;
        }
        ism330dhcx_accel_range_t accel_range = ISM330DHCX_ACCEL_RANGE_4_G;
        switch (accel_range)
        {
        case ISM330DHCX_ACCEL_RANGE_16_G: accel_scale = 0.48828125;
            break;
        case ISM330DHCX_ACCEL_RANGE_8_G: accel_scale = 0.244140625;
            break;
        case ISM330DHCX_ACCEL_RANGE_4_G: accel_scale = 0.1220703125;
            break;
        case ISM330DHCX_ACCEL_RANGE_2_G: accel_scale = 0.06103515625;
            break;      
        default:
            break;
        }

        // initiate CF, run for 1 second
        bool _initiating = true;
        
        if (debug){
            printf("Initiating complemetary filter...\n");
        }
        
        uint64_t start_time = micros();
        t_final = start_time;
        while (_initiating){
            readIMU(); // Main CF function inside
            if ((micros()-start_time)>1000000) {
                if (debug){
                    printf("Complementary filter initiated.\n");
                }
            _initiating = false;
        }
        while ((micros()-t_final) < (*dt).floatingPoint) {} // Keep loop at dt
        t_final = micros();
        }
    }
}

/*******************************************************************************
 * Main
 */
void loop() {
    float loop_time = 0;
    // Loop forever
    while (true)
    {
        readIMU();
        readContact();
        dump();
        loop_time = (micros()-t_final);
        // printf("Loop Time: %10.2f\r\n", loop);
        if (loop_time > (*dt).floatingPoint){
            error = error|0b00000001; 
            gpio_put(LED_PIN, false);
        }
        else{
            gpio_put(LED_PIN, true);
            error = error&0b11111110;
        }
        //loop time control  
        while ((micros()-t_final) < (*dt).floatingPoint) {} // Keep loop at dt
        t_final = micros();
    }   
        
}
