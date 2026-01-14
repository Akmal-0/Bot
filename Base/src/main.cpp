#include <Arduino.h>
#include <config.h>
#include <pid.h>
#include <kinematic.h>
#include <odometry.h>
#include <imu.h>
#include <USBHost_t36.h>

// void readEncoder0();
// void readEncoder1();
// void readEncoder2();
// void readEncoder3();
template <int j>
void readEncoder();
void setMotor(int cwPin, int ccwPin, float pwmVal);
void button_list(uint32_t buttons, int joy_axis);
void moveBase();
void Emergencystop();
void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_);
void gripperstaff();
void gripperSequence();
void gripper();
void gripperfront();
void handleSpeedToggle();
void printButtonActive();
void printAxis();

USBHost usb_joy;
USBHub joy_hub(usb_joy);
USBHIDParser hid_joy(usb_joy);

#define COUNT_JOYSTICKS 4

JoystickController joy_control[COUNT_JOYSTICKS]{
	JoystickController(usb_joy), JoystickController(usb_joy),
	JoystickController(usb_joy), JoystickController(usb_joy)};
int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = {&joy_hub, &joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3], &hid_joy};
#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]", "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};
bool show_changed_only = false;

uint8_t joystick_left_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_right_trigger_value[COUNT_JOYSTICKS] = {0};
uint64_t joystick_full_notify_mask = (uint64_t)-1;

unsigned long prev_odom_update = 0;
unsigned long prevT = 0;

unsigned long lastJoystickTime = 0;
const unsigned long JOYSTICK_TIMEOUT = 300;

// bool speedModeFast = false;    
// bool lastButtonUp = false;

static bool speedModeFast = false;
static bool lastButtonUp = false;
static unsigned long lastToggleTime = 0;

bool smoothed = false;
// unsigned long prevT = 0;



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
volatile long pos[4];

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematic kinematic(
	Kinematic::ROBOT_1,
	MOTOR_MAX_RPS,
	MAX_RPS_RATIO,
	MOTOR_OPERATING_VOLTAGE,
	MOTOR_POWER_MAX_VOLTAGE,
	WHEEL_DIAMETER,
	ROBOT_DIAMETER);

Odometry odometry;

IMU imu_sensor;

void setup()
{
	Serial.begin(115200);
	usb_joy.begin();

	bno.begin();
	bno.setExtCrystalUse(true);

	// joy_control[0].joystickType();

	pinMode(sol, OUTPUT);
	pinMode(solgrip, OUTPUT);
	pinMode(solpush,OUTPUT);
	pinMode(solfront,OUTPUT);

	for (int i = 0; i < 4; i++) // Ubah dari 3 ke 4 untuk 4 motor
	{
		pinMode(cw[i], OUTPUT);
		pinMode(ccw[i], OUTPUT);

		analogWriteFrequency(cw[i], PWM_FREQUENCY);
		analogWriteFrequency(ccw[i], PWM_FREQUENCY);

		analogWriteResolution(PWM_BITS);
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);

		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);
	}

	wheel1.ppr_total(COUNTS_PER_REV1);
	wheel2.ppr_total(COUNTS_PER_REV2);
	wheel3.ppr_total(COUNTS_PER_REV3);
	wheel4.ppr_total(COUNTS_PER_REV4);

	// attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder0, RISING);
	// attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder1, RISING);
	// attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder2, RISING);
	// attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder3, RISING);

	attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);

	pinMode(LED_PIN, OUTPUT);
}

int applyDeadzone(int value, int deadzone = 15) //10
{
	if (abs(value) < deadzone)
	{
		return 0;
	}
	return value;
}

void loop()
{
	usb_joy.Task();

	if (!joy_control[0])
	{
		Emergencystop();
		lastJoystickTime = 0;
		delay(100); 
		return;
	}
	if (joy_control[0].available())
	{

		lastJoystickTime = millis();

		uint32_t buttons = joy_control[0].getButtons();
		button_list(buttons, joy_control[0].getAxis(9));

		joystick.axis1_x = applyDeadzone(joy_control[0].getAxis(1) - 128);
		joystick.axis1_y = applyDeadzone(joy_control[0].getAxis(0) - 128);
		joystick.axis0_x = applyDeadzone(joy_control[0].getAxis(5) - 128);
		joystick.axis0_y = applyDeadzone(joy_control[0].getAxis(2) - 128);

		handleSpeedToggle();
		moveBase();
		gripperstaff();
		gripperSequence();
		gripper();
		gripperfront();
		// printButtonActive();
		// printAxis();
	}

	if (millis() - lastJoystickTime > JOYSTICK_TIMEOUT)
	{
		Emergencystop();
	}
}

joy joySmoothed;
joy joyCmd;
// bool smoothed = false;

void moveBase()
{
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

	unsigned long currT = micros();
	float deltaT = ((float)(currT - prevT)) / 1.0e6;

	// 	if (button.up == 1 && !lastButtonUp)
    // {
    //     speedModeFast = !speedModeFast;
    // }
    // lastButtonUp = button.up;

    float top_speed;
    float rot_speed;

    if (speedModeFast)
    {
        //fast
        top_speed = 1.2;
        rot_speed = 0.9;
    }
    else
    {
        //slow
        top_speed = 0.5;
        rot_speed = 0.3;
    }

	// float top_speed = 1.5;
	// float rot_speed = 1.0;

	joystick.axis1_x = map(-joystick.axis1_x, 0, 128, 0, top_speed);
	joystick.axis1_y = map(joystick.axis1_y, 0, 128, 0, top_speed);
	joystick.axis0_y = map(joystick.axis0_y, 0, 128, 0, rot_speed);

	//FLOAT SAFE
	// joystick.axis1_x = (-joystick.axis1_x / 128.0) * top_speed;
	// joystick.axis1_y = ( joystick.axis1_y / 128.0) * top_speed;
	// joystick.axis0_y = ( joystick.axis0_y / 128.0) * rot_speed;

	joyCmd.axis1_x = joystick.axis1_x;
	joyCmd.axis1_y = joystick.axis1_y;
	joyCmd.axis0_y = joystick.axis0_y;


	calculate_smooth_vel(joySmoothed.axis1_x, joystick.axis1_x, deltaT, 1.0);
	calculate_smooth_vel(joySmoothed.axis1_y, joystick.axis1_y, deltaT, 1.0);
	calculate_smooth_vel(joySmoothed.axis0_y, joystick.axis0_y, deltaT, 1.0);

	// if (button.left == 1)
	// {
	// 	smoothed = true;
	// }
	// else if (button.right == 1)
	// {
	// 	smoothed = false;
	// }
	// if (smoothed)
	// {
	// 	joyCmd.axis1_x = joySmoothed.axis1_x;
	// 	joyCmd.axis1_y = joySmoothed.axis1_y;
	// 	joyCmd.axis0_y = joySmoothed.axis0_y;

	// 	joyCmd.axis1_x = joystick.axis1_x;
	// 	joyCmd.axis1_y = joystick.axis1_y;
	// 	joyCmd.axis0_y = joystick.axis0_y;
	// }
	// else if (!smoothed)
	// {
	// 	joyCmd.axis1_x = joystick.axis1_x;
	// 	joyCmd.axis1_y = joystick.axis1_y;
	// 	joyCmd.axis0_y = joystick.axis0_y;
	// }

	Kinematic::rps req_rps;
	req_rps = kinematic.getRPS(
		joyCmd.axis1_x,
		-joyCmd.axis1_y,
		joyCmd.axis0_y,
		-event.orientation.x);

	float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
	float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
	float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);
	float controlled_motor4 = wheel4.control_speed(req_rps.motor4, pos[3], deltaT);

	float current_rps1 = wheel1.get_filt_vel();
	float current_rps2 = wheel2.get_filt_vel();
	float current_rps3 = wheel3.get_filt_vel();
	float current_rps4 = wheel4.get_filt_vel();

	// Serial.print(" | ");
	// Serial.print(pos[0]);
	// Serial.print(" | ");
	// Serial.print(pos[1]);
	// Serial.print(" | ");
	// Serial.print(pos[2]);
	// Serial.print(" | ");
	// Serial.print(pos[3]);
	// Serial.print(" | ");

	if (fabs(req_rps.motor1) < 0.02)
	{
		controlled_motor1 = 0.0;
	}
	if (fabs(req_rps.motor2) < 0.02)
	{
		controlled_motor2 = 0.0;
	}
	if (fabs(req_rps.motor3) < 0.02)
	{
		controlled_motor3 = 0.0;
	}
	if (fabs(req_rps.motor4) < 0.02)
	{
		controlled_motor4 = 0.0;
	}

	setMotor(cw[0], ccw[0], controlled_motor1);
	setMotor(cw[1], ccw[1], controlled_motor2);
	setMotor(cw[2], ccw[2], controlled_motor3);
	setMotor(cw[3], ccw[3], controlled_motor4);

	Kinematic::velocities vel = kinematic.getVelocities(
		current_rps1,
		current_rps2,
		current_rps3,
		current_rps4);

	unsigned long now = millis();
	float vel_dt = (now - prev_odom_update) / 1000.0;
	prev_odom_update = now;
	
	odometry.update(
		vel_dt,
		vel.linear_x,
		vel.linear_y,
		vel.angular_z);
	Odometry::odom robot_pose;
	robot_pose = odometry.getData();
	// Serial.print(" | ");
	// Serial.print(joySmoothed.axis1_x);
	// Serial.print(" | ");
	// Serial.print(joySmoothed.axis1_y);
	// Serial.print(" | ");
	// Serial.print(joySmoothed.axis0_y);
	// Serial.print(" | ");
	// Serial.print(req_rps.motor1);
	// Serial.print(" | ");
	// Serial.print(req_rps.motor2);
	// Serial.print(" | ");
	// Serial.print(req_rps.motor3);
	// Serial.print(" | ");
	// Serial.print(req_rps.motor4);
	// Serial.print(" | ");

	prevT = currT;
}


void handleSpeedToggle()
{
    const unsigned long debounceDelay = 150;

    if (button.up &&
        !lastButtonUp &&
        millis() - lastToggleTime > debounceDelay)
    {
        speedModeFast = !speedModeFast;
        lastToggleTime = millis();
    }

    lastButtonUp = button.up;
}

bool last_call=false;
void handle_gear(){
	if (button.up && !last_call){
		speedModeFast = true;
		last_call = true;

	}else if(button.up && last_call){
		speedModeFast = false;
		last_call = false;
		
	}
}

void Emergencystop()
{
	for (int i = 0; i < 4; i++)
	{
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);
	}
}

void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_)
{
	double target = input_value;

	if (input_value == 0.0 && std::abs(current_output) > 1e-6)
	{
		target = 0.0;
	}
	double max_delta = max_acceleration_ * deltaT;

	double delta_target = target - current_output;

	if (std::abs(delta_target) > max_delta)
	{
		delta_target = (delta_target > 0.0) ? max_delta : -max_delta;
	}

	current_output += delta_target;
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
	if (pwmVal > 0)
	{
		analogWrite(cwPin, fabs(pwmVal));
		analogWrite(ccwPin, 0);
	}
	else if (pwmVal < 0)
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, fabs(pwmVal));
	}
	else
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, 0);
	}
}

template <int j>
void readEncoder()
{
	int b = digitalRead(encb[j]);
	if (b > 0)
	{
		pos[j]++;
	}
	else
	{
		pos[j]--;
	}
}

void gripperstaff()
{
	static bool lastButtonState = 0;
	static bool solenoidState = 0;

	bool currentButton = button.RT;

	if (currentButton == 1 && lastButtonState == 0)
	{
		solenoidState = !solenoidState;
		digitalWrite(sol, solenoidState);
	}

	lastButtonState = currentButton;
}

void gripperSequence()
{
	static bool lastButtonState = 0;
	static bool solenoidState = 0;

	bool currentButton = button.RB;

	if (currentButton == 1 && lastButtonState == 0)
	{
		solenoidState = !solenoidState;
		digitalWrite(solgrip, solenoidState);
	}

	lastButtonState = currentButton;
}

void gripper()
{
	static bool lastButtonState = 0;
	static bool solenoidState = 0;

	bool currentButton = button.A;

	if (currentButton == 1 && lastButtonState == 0)
	{
		solenoidState = !solenoidState;
		digitalWrite(solpush, solenoidState);
	}

	lastButtonState = currentButton;
}

void gripperfront()
{
	static bool lastButtonState = 0;
	static bool solenoidState = 0;

	bool currentButton = button.B;

	if (currentButton == 1 && lastButtonState == 0)
	{
		solenoidState = !solenoidState;
		digitalWrite(solfront, solenoidState);
	}

	lastButtonState = currentButton;
}

void button_list(uint32_t buttons, int joy_axis)
{
	// RESET
	button.A = button.B = button.X = button.Y = 0;
	button.LB = button.RB = button.LT = button.RT = 0;
	button.start = button.select = button.home = 0;

	// BUTTON 
	if (buttons & 2)    button.A = 1;
	if (buttons & 4)    button.B = 1;
	if (buttons & 1)    button.X = 1;
	if (buttons & 8)    button.Y = 1;
	if (buttons & 32)   button.RB = 1;
	if (buttons & 16)   button.LB = 1;
	if (buttons & 64)   button.LT = 1;
	if (buttons & 128)  button.RT = 1;
	if (buttons & 256)  button.select = 1;
	if (buttons & 512)  button.start = 1;
	if (buttons & 4096) button.home = 1;

	// DPAD / HAT
	button.up = button.down = button.left = button.right = 0;

	switch (joy_axis)
	{
	case 0: button.up = 1; break;
	case 2: button.right = 1; break;
	case 4: button.down = 1; break;
	case 6: button.left = 1; break;
	}
}

// void printButtonActive()
// {
//   if (button.A) Serial.println(" A Press");
//   if (button.B) Serial.println(" B Press");
//   if (button.X) Serial.println(" X Press");
//   if (button.Y) Serial.println(" Y Press");

//   if (button.LB) Serial.println(" LB Press");
//   if (button.RB) Serial.println(" RB Press");
//   if (button.LT) Serial.println(" LT Press");
//   if (button.RT) Serial.println(" RT Press");

//   if (button.start) Serial.println("START");
//   if (button.select) Serial.println("SELECT");
//   if (button.home) Serial.println("HOME");

//   if (button.up) Serial.println("UP");
//   if (button.down) Serial.println("DOWN");
//   if (button.left) Serial.println("LEFT");
//   if (button.right) Serial.println("RIGHT");
// }

// void printAxis()
// {
//   Serial.print("AXIS | ");

//   Serial.print("Lx: ");
//   Serial.print(joy_control[0].getAxis(1) - 128);
//   Serial.print(" | Ly: ");
//   Serial.print(joy_control[0].getAxis(0) - 128);

//   Serial.print(" | Rx: ");
//   Serial.print(joy_control[0].getAxis(5) - 128);
//   Serial.print(" | Ry: ");
//   Serial.print(joy_control[0].getAxis(2) - 128);

//   Serial.print(" | HAT: ");
//   Serial.print(joy_control[0].getAxis(9));

//   Serial.println();
// }

