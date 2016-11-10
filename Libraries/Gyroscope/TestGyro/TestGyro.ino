/*
 Name:		TestGyro.ino
 Created:	4/19/2016 3:47:36 PM
 Author:	Nemanja
*/
#include "Gyroscope.h"

Gyroscope gyro(9);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {
	Serial.begin(38400);
	attachInterrupt(0, dmpDataReady, RISING);
	gyro.bootUp();
}


void loop() {
	while (!mpuInterrupt){
		
	}
	if (mpuInterrupt) GyroRead(&gyro,mpuInterrupt);
}
