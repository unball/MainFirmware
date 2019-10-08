#include <Arduino.h>

#define TEENSY_DEBUG false
#define ROBOT_NUMBER 0
#define scale 1000
#include "radio.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "control.hpp"


Radio::dataStruct vel;
Imu::imuAll imuData;
double pos(double angVel);
double filter(double var);
void setup() {
	//Serial.begin(9600);
	//while(!Serial);
	#if (TEENSY_DEBUG || CONTROL_DEBUG || IMU_DEBUG || MOTOR_DEBUG)
	Serial.println("SETUP!");
	#endif
	Radio::setup(0, 3);
	Imu::Setup();
	Motor::setup();
	Encoder::setup();
}

int8_t wave_flag = 1;
int8_t triangular_incrementer = 1;


#define DEADZONE 10
int32_t deadzone(int32_t vin){
	if (vin!=0)
		return (vin > 0) ? vin+DEADZONE : vin-DEADZONE;
	return 0;
}

double saturation(double vin){
	return min(max(vin, -255.0), 255.0);
}	

void triangular_wave(int32_t *v1, int32_t *v2){
	static int32_t triangular_wave_cont;
	if(triangular_wave_cont >= scale){
		triangular_incrementer = -1;
	}
	else if(triangular_wave_cont <= -scale){
		triangular_incrementer = 1;
	}
	triangular_wave_cont += triangular_incrementer;
	*v1 = triangular_wave_cont*64/scale;
	*v2 = triangular_wave_cont*64/scale;
}

void sine_wave(int32_t *v1, int32_t *v2){
	static int32_t sine_wave_cont;
	if(sine_wave_cont >= 100000){
		sine_wave_cont = 0;
	}
	sine_wave_cont ++;
	*v1 = 40*sin(sine_wave_cont/400.0);
	*v2 = 40*sin(sine_wave_cont/400.0);
}

void square_wave(int32_t *v1, int32_t *v2){
	static uint32_t square_wave_cont;
	if(square_wave_cont > 500){
		wave_flag = -1*wave_flag;
		square_wave_cont = 0;
	}
	*v1 = 20*wave_flag;
	*v2 = 20*wave_flag;
	square_wave_cont++;
}

void step(int32_t *v1, int32_t *v2){
	static uint32_t step_cont;
	static int8_t step_flag = 0;
	if(step_cont > 200){
		step_flag = 1;
	}
	
	if(step_cont>2000){
		step_flag = 0;
		
	}
	step_cont++;
	*v1 = 30*step_flag;
	*v2 = 30*step_flag;
}

void run_straight(int32_t *v1, int32_t *v2){
	*v1 = 64;
	*v2 = 64;
}

double control1(double err){
	static double old_err;
	static double old_out;
	double out = (  1.9077 * (err - 0.9418  *  old_err) + old_out);
	old_err = err -  (saturation(out)-out);
	old_out = out; //=  (abs(out) < 255)? out : 0;
	return out;
}

double control2(double err){
	static double old_err;
	static double old_out;
	double out =  (   1.9077 * (err  - 0.9418 *  old_err) + old_out);
	old_err = err - (saturation(out)-out);
	old_out = out; //(abs(out) < 255)? out : 0;;
	return out;
}

float err1 =0, err2 = 0, vela=0;

void loop() {
	static int32_t previous_t;
	static int32_t t;

	t = micros();

	if(t-previous_t >= 1500){
		//Serial.println(t-previous_t);
		int32_t v1,v2;
		triangular_wave(&v1, &v2);

		previous_t = t;
		//Motor::move(0, deadzone(v1));
		//Motor::move(1, deadzone(v2));

		Encoder::vel enc = Encoder::encoder();
		Radio::vel message;
		
		vela =enc.motorA;
		err1 = v1 - vela  - 1* (vela-enc.motorB);
		err2 = v2 - enc.motorB +  1*(vela-enc.motorB) ;
		int32_t controlA = (int32_t)control1(err1);
		int32_t controlB = (int32_t)control2(err2);
		// Serial.print(err1);
		// Serial.print("\t");
		// Serial.print(controlA);
		// Serial.print("\t");
		// Serial.print(err2);
		// Serial.print("\t");
		// Serial.print(controlB);
		// Serial.println("\r");
		Motor::move(0, deadzone(controlA));
		Motor::move(1, deadzone(controlB));

		message.vel_A = (int32_t)vela;
		message.vel_B = (int32_t)enc.motorB;
		message.in_A = v1;
		message.in_B = v2;
		message.time = t;

		Radio::reportMessage(message);
	}
	
	//delay(1);

	#if TEENSY_DEBUG
	Serial.println("LOOP!");
	
	//=========Radio==========
	vel.B = 56;
	vel.A = 47;
	Radio::receiveData(&vel);
	Serial.println("Radio:");
	Serial.print("a: ");Serial.print(vel.A);Serial.print("\tb: ");Serial.println(vel.B);
	//=========End Radio===========

	//=========IMU==========  
	Serial.println("IMU:");
	imuData = Imu::imuRead();
	Serial.println("accel:");
	Serial.print("x: ");Serial.println(imuData.accel.x);
	Serial.print("y: ");Serial.println(imuData.accel.y);
	Serial.print("z: ");Serial.println(imuData.accel.z);
	Serial.println("gyro:");
	Serial.print("x: ");Serial.println(imuData.gyro.x);
	Serial.print("y: ");Serial.println(imuData.gyro.y);
	Serial.print("z: ");Serial.println(imuData.gyro.z);
	//=========End IMU==========

	//=========Motor==========
	Motor::move(0, 100);
	Motor::move(1, 100);
	//=========End Motor==========

	//=========Encoder==========
	Serial.println("Encoder:");
	Encoder::vel enc;
	enc = Encoder::encoder();
	Serial.print("Channel A: ");Serial.println(enc.motorA);
	Serial.print("Channel B: ");Serial.println(enc.motorB);
	//=========End Encoder==========

	delay(100);
	#else
	//Control::stand();
	//delay(1);
	#endif
}

double pos(double angVel){
	static double pos = 0, oldVel = 0;
	pos = pos + ((angVel + oldVel)/2.0)*(Imu::deltaT()/1000000.0);
	oldVel = angVel;
	return pos;
}

double filter(double var){
	static double varOld;
	varOld = 0.05*var + 0.95*varOld;
	return varOld;
}