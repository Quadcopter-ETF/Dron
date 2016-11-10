#pragma once
#ifndef _GYRO_h
#define _GYRO_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// ------------------------MUST-HAVE LIBS-----------------------------
// This libraries must be present in Arduino/Libraries folder. 
// They can be found here https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Filters.h"
#include "MovingAVG.h"
// -------------------------------------------------------------------

class Gyroscope{
public:
	friend void GyroRead(Gyroscope *g, volatile bool& mpuInterrupt);
	// ------------------------Config for Calibration-----------------------------
	int16_t ax, ay, az, gx, gy, gz;
	int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
	int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
	// Change this 3 variables if you want to fine tune the skecth to your needs:

	// Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	int buffersize = 1000;
	// Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	int acel_deadzone = 8;
	// Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
	int giro_deadzone = 1;

	// calibration functions:
	void meansensors();
	void calibration();
	// ---------------------------------------------------------------------------

	// ------------------------Gyro control/status vars---------------------------
	bool blinkState = false;
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from Gyro
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	float filterFrequency = 100;
	float windowLength = 20.0 / filterFrequency;
	// ---------------------------------------------------------------------------

	// ------------------------Orientation/Motion vars---------------------------- 
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	// filter
	FilterOnePole yawFilter;
	FilterOnePole pitchFilter;
	FilterOnePole rollFilter;
	FilterOnePole aXFilter;
	FilterOnePole aYFilter;
	FilterOnePole aZFilter;
	RunningStatistics yawStats;
	RunningStatistics pitchStats;
	RunningStatistics rollStats;
	RunningStatistics aXStats;
	RunningStatistics aYStats;
	RunningStatistics aZStats;
	MovingAvgFloat yawAngle;
	MovingAvgFloat pitchAngle;
	MovingAvgFloat rollAngle;
	MovingAvgFloat aXAccel;
	MovingAvgFloat aYAccel;
	MovingAvgFloat aZAccel;
	// ---------------------------------------------------------------------------

//public:
	// ------------------------Class variables------------------------------------
	MPU6050 gyro;
	uint8_t RTD; // RTD represents Ready-To-Drive buzzer/LED which signals whether the MPU6050 is calibrated and safe to use
	// ---------------------------------------------------------------------------

	// ------------------------Class methods--------------------------------------
	Gyroscope(uint8_t pinRTD = 15); // Class constructor
	void bootUp(); // boot-up routine
	void readWorldAccel(); // this method returns World Acceleration values
	void readAccel(float&,float&,float&); // this method returns Acceleration values
	void readYPR(float&,float&,float&); // this method returns Yaw, Pitch, Roll values
	void readEuler(float&,float&,float&); // this method returns Euler values
	void checkFIFOOverFlow();
	// ---------------------------------------------------------------------------
};

inline void GyroRead(Gyroscope *g, volatile bool& mpuInterrupt, float& yaw1,float& pitch1,float& roll1){
	mpuInterrupt = false;
	g->mpuIntStatus = g->gyro.getIntStatus();
	g->mpuIntStatus = true;
	g->fifoCount = g->gyro.getFIFOCount();
	while (g->fifoCount < g->packetSize) g->fifoCount = g->gyro.getFIFOCount();
	g->gyro.getFIFOBytes(g->fifoBuffer, g->packetSize);
	g->gyro.resetFIFO();
	g->fifoCount -= g->packetSize;
	g->readYPR(yaw1,pitch1,roll1);
}
#endif