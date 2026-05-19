#ifndef ROBOT_1_CONFIG_H
#define ROBOT_1_CONFIG_H

#define LED 13
#define IR_SEND_PIN 41

// uncomment the base you're building
// #define ROBOT_2 DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define ROBOT_2 SKID_STEER              // 4WD robot
// #define ROBOT_2 MECANUM                // Mecanum drive robot
#define ROBOT_1 OMNI // Omniwheel robot

// uncomment the motor driver you're using
// #define USE_GENERIC_2_IN_MOTOR_DRIVER         // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER        // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER               // Motor ESC for brushless motors

// uncomment the IMU you're using
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
#define USE_BNO055_IMU

#define K_P 90            // 55
#define K_I 366.101694915 // 0.045454545
#define K_D 0             // 0

// #define K_P 2.43            // 55
// #define K_I 48.60 // 0.045454545
// #define K_D 0.03            // 0

// define your robot' specs here
#define MOTOR_MAX_RPS 8.4               // motor's max RPM
#define MAX_RPS_RATIO 0.85              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 3840            // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 3840            // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 3840            // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 3840            // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0985           // wheel's diameter in meters
#define ROBOT_DIAMETER 0.80             // 800       // distance between left and right wheels
#define ROBOT_RADIUS 0.40               // 400
#define PWM_BITS 8
#define PWM_FREQUENCY 25000 // 1000

// // INVERT ENCODER COUNTS
// #define MOTOR1_ENCODER_INV false
// #define MOTOR2_ENCODER_INV false
// #define MOTOR3_ENCODER_INV false
// #define MOTOR4_ENCODER_INV false

// // INVERT MOTOR DIRECTIONS
// #define MOTOR1_INV false
// #define MOTOR2_INV false
// #define MOTOR3_INV false
// #define MOTOR4_INV false

// ENCODER PINS
#define MOTOR1_ENCODER_A 21 // 34 //14
#define MOTOR1_ENCODER_B 20 // 35 //15

#define MOTOR2_ENCODER_A 0 // 38 //17
#define MOTOR2_ENCODER_B 1 // 39 //16

#define MOTOR3_ENCODER_A 26 // 40 //37   //29
#define MOTOR3_ENCODER_B 34 // 41 //29    //37

#define MOTOR4_ENCODER_A 28 // 18 //36
#define MOTOR4_ENCODER_B 27 // 19 //27

// MOTOR PINS
#define MOTOR1_IN_A 14 // 23 //5
#define MOTOR1_IN_B 15 // 22 //6

#define MOTOR2_IN_A 37 // 18
#define MOTOR2_IN_B 36 // 19

#define MOTOR3_IN_A 29 // 3
#define MOTOR3_IN_B 33 // 4

#define MOTOR4_IN_A 6 // 22
#define MOTOR4_IN_B 5 // 10

#define pinIN1 22 // grip box
#define pinIN2 23 // grip box

#define motorpush 12 // 3 // push gripper
#define motorpull 11 // 4

#define motorup 3   // 12   // grip up down
#define motordown 4 // 11 // grip up down

#define limit2 25
#define limit1 40

#define limit4 24
#define limit3 39

// encoder in array
const int enca[4] = {
    MOTOR1_ENCODER_A,
    MOTOR2_ENCODER_A,
    MOTOR3_ENCODER_A,
    MOTOR4_ENCODER_A};
const int encb[4] = {
    MOTOR1_ENCODER_B,
    MOTOR2_ENCODER_B,
    MOTOR3_ENCODER_B,
    MOTOR4_ENCODER_B};

// motor in array
const int cw[4] = {
    MOTOR1_IN_A,
    MOTOR2_IN_A,
    MOTOR3_IN_A,
    MOTOR4_IN_A};
const int ccw[4] = {
    MOTOR1_IN_B,
    MOTOR2_IN_B,
    MOTOR3_IN_B,
    MOTOR4_IN_B};

// IMU 18,19 / SDA0,SCL0
// IMU 16,17 / SDA1,SCL1

int sol1 = 38;
int sol2 = 10;
int sol3 = 18;
int sol4 = 19;

// const int pwm[4] ={-1,-1,-1,-1};
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

struct but
{
    int A;
    int B;
    int X;
    int Y;
    int RT;
    int LT;
    int LB;
    int RB;
    int select;
    int start;
    int home;
    int up;
    int down;
    int left;
    int right;
} button;
struct joy
{
    double axis0_x;
    double axis0_y;
    double axis1_x;
    double axis1_y;
    int but_red;
    int but_blue;
    int but_black;
    int but_green;
} joystick;

#endif