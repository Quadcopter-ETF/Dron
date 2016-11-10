#include "Gyroscope.h"

Gyroscope::Gyroscope(uint8_t pinRTD){
	gyro = MPU6050();
	RTD = pinRTD;
	pinMode(RTD, OUTPUT);
	digitalWrite(RTD, LOW);
	yawFilter = FilterOnePole(LOWPASS, filterFrequency);
	pitchFilter = FilterOnePole(LOWPASS, filterFrequency);
	rollFilter = FilterOnePole(LOWPASS, filterFrequency);
	yawStats = RunningStatistics();
	pitchStats = RunningStatistics();
	rollStats = RunningStatistics();
	yawStats.setWindowSecs(windowLength);
	pitchStats.setWindowSecs(windowLength);
	rollStats.setWindowSecs(windowLength);
}

void Gyroscope::bootUp(){
	// boot-up routine

	// The buzzer will quickly pulse on and off
	for (int i = 0; i < 5; i++){
		digitalWrite(RTD, HIGH);
		delay(300);
		digitalWrite(RTD, LOW);
		delay(300);
	}

	// ------------------------I2C initialization---------------------------------
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	// comment the line bellow if Arduino DUE is used
	 //TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	gyro.initialize();
	// ---------------------------------------------------------------------------

	// check if the Gyroscope is successfully connected
	if (gyro.testConnection()) {
		for (int i = 0; i < 3; i++){
			digitalWrite(RTD, HIGH);
			delay(50);
			digitalWrite(RTD, LOW);
			delay(50);
		}
	}
	else {
		while (1){
			digitalWrite(RTD, HIGH);
		};
	}

	// ------------------------Calibration----------------------------------------
	delay(1000);
	// reset offsets
	gyro.setXAccelOffset(0);
	gyro.setYAccelOffset(0);
	gyro.setZAccelOffset(0);
	gyro.setXGyroOffset(0);
	gyro.setYGyroOffset(0);
	gyro.setZGyroOffset(0);
	while (1){
		if (state == 0){
			meansensors();
			state++;
			digitalWrite(RTD, HIGH);
			delay(20);
			digitalWrite(RTD, LOW);
			delay(50);
		}
		else if (state == 1) {
			calibration();
			state++;
			digitalWrite(RTD, HIGH);
			delay(20);
			digitalWrite(RTD, LOW);
			delay(50);
		}

		else if (state == 2) {
			meansensors();
			for (int i = 0; i < 5; i++){
				digitalWrite(RTD, HIGH);
				delay(20);
				digitalWrite(RTD, LOW);
				delay(20);
			}
			delay(50);
			break;
		}
	}
	// ---------------------------------------------------------------------------

	// ------------------------DMP------------------------------------------------
	devStatus = gyro.dmpInitialize();
	// Setting the offsets based on previous calculation
	gyro.setXGyroOffset(gx_offset);
	gyro.setYGyroOffset(gy_offset);
	gyro.setZGyroOffset(gz_offset);
	gyro.setXAccelOffset(ax_offset);
	gyro.setYAccelOffset(ay_offset);
	gyro.setZAccelOffset(az_offset);
	digitalWrite(RTD, HIGH);
	delay(20);
	digitalWrite(RTD, LOW);
	
	if (devStatus == 0) {
		digitalWrite(RTD, HIGH);
		delay(20);
		digitalWrite(RTD, LOW);
		gyro.setDMPEnabled(true);
		dmpReady = true;
		packetSize = gyro.dmpGetFIFOPacketSize();
	}
	else{
		while (1){
			digitalWrite(RTD, HIGH);
		};
	}	
	// ---------------------------------------------------------------------------

	// Gyroscope is set
	for (int i = 0; i < 5; i++){
		digitalWrite(RTD, HIGH);
		delay(200);
		digitalWrite(RTD, LOW);
		delay(200);
	}
	for (int i = 0; i < 10; i++){
		digitalWrite(RTD, HIGH);
		delay(20);
		digitalWrite(RTD, LOW);
		delay(20);
	}
}

void Gyroscope::meansensors(){
	long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

	while (i<(buffersize + 101)){
		// read raw accel/gyro measurements from device
		gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		if (i>100 && i <= (buffersize + 100)){ //First 100 measures are discarded
			buff_ax = buff_ax + ax;
			buff_ay = buff_ay + ay;
			buff_az = buff_az + az;
			buff_gx = buff_gx + gx;
			buff_gy = buff_gy + gy;
			buff_gz = buff_gz + gz;
		}
		if (i == (buffersize + 100)){
			mean_ax = buff_ax / buffersize;
			mean_ay = buff_ay / buffersize;
			mean_az = buff_az / buffersize;
			mean_gx = buff_gx / buffersize;
			mean_gy = buff_gy / buffersize;
			mean_gz = buff_gz / buffersize;
		}
		i++;
		delay(2); //Needed so we don't get repeated measurements
	}
}

void Gyroscope::calibration(){
	ax_offset = -mean_ax / 8;
	ay_offset = -mean_ay / 8;
	az_offset = (16384 - mean_az) / 8;

	gx_offset = -mean_gx / 4;
	gy_offset = -mean_gy / 4;
	gz_offset = -mean_gz / 4;
	while (1){
		int ready = 0;
		gyro.setXAccelOffset(ax_offset);
		gyro.setYAccelOffset(ay_offset);
		gyro.setZAccelOffset(az_offset);

		gyro.setXGyroOffset(gx_offset);
		gyro.setYGyroOffset(gy_offset);
		gyro.setZGyroOffset(gz_offset);
		meansensors();
		digitalWrite(RTD, HIGH);
		delay(20);
		digitalWrite(RTD, LOW);
		if (abs(mean_ax) <= acel_deadzone) ready++;
		else ax_offset = ax_offset - mean_ax / acel_deadzone;

		if (abs(mean_ay) <= acel_deadzone) ready++;
		else ay_offset = ay_offset - mean_ay / acel_deadzone;

		if (abs(16384 - mean_az) <= acel_deadzone) ready++;
		else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

		if (abs(mean_gx) <= giro_deadzone) ready++;
		else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

		if (abs(mean_gy) <= giro_deadzone) ready++;
		else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

		if (abs(mean_gz) <= giro_deadzone) ready++;
		else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

		if (ready == 6) break;
	}
}

void Gyroscope::readWorldAccel(){
	gyro.dmpGetQuaternion(&q, fifoBuffer);
	gyro.dmpGetAccel(&aa, fifoBuffer);
	gyro.dmpGetGravity(&gravity, &q);
	gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	gyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	Serial.print("aworld\t");
	Serial.print(aaWorld.x);
	Serial.print("\t");
	Serial.print(aaWorld.y);
	Serial.print("\t");
	Serial.println(aaWorld.z);
}

void Gyroscope::readAccel(float& X,float& Y,float& Z){
	gyro.dmpGetQuaternion(&q, fifoBuffer);
	gyro.dmpGetAccel(&aa, fifoBuffer);
	gyro.dmpGetGravity(&gravity, &q);
	gyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	aXFilter.input(aaReal.x);
	aYFilter.input(aaReal.y);
	aZFilter.input(aaReal.z);
	aXStats.input(aXFilter.output());
	aYStats.input(aYFilter.output());
	aZStats.input(aZFilter.output());
	aXAccel.add(aXStats.mean());
	aYAccel.add(aYStats.mean());
	aZAccel.add(aZStats.mean());
	X=aXAccel.read();
	Y=aYAccel.read();
	Z=aZAccel.read();
}

void Gyroscope::readEuler(float& aX,float& aY,float& aZ){
	gyro.dmpGetQuaternion(&q, fifoBuffer);
	gyro.dmpGetEuler(euler, &q);
	Serial.print("euler\t");
	Serial.print(euler[0] * 180 / M_PI);
	Serial.print("\t");
	Serial.print(euler[1] * 180 / M_PI);
	Serial.print("\t");
	Serial.println(euler[2] * 180 / M_PI);
	aXFilter.input(aaReal.x);
	aYFilter.input(aaReal.y);
	aZFilter.input(aaReal.z);
	aXStats.input(aXFilter.output());
	aYStats.input(aYFilter.output());
	aZStats.input(aZFilter.output());
	aXAccel.add(aXStats.mean());
	aYAccel.add(aYStats.mean());
	aZAccel.add(aZStats.mean());
	aX=aXAccel.read();
	aY=aYAccel.read();
	aZ=aZAccel.read();
}

void Gyroscope::readYPR(float& yaw2,float& pitch2, float& roll2){
	gyro.dmpGetQuaternion(&q, fifoBuffer);
	gyro.dmpGetGravity(&gravity, &q);
	gyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
	// filtering
	yawFilter.input(ypr[0] * 180 / M_PI);
	pitchFilter.input(ypr[1] * 180 / M_PI);
	rollFilter.input(ypr[2] * 180 / M_PI);
	yawStats.input(yawFilter.output());
	pitchStats.input(pitchFilter.output());
	rollStats.input(rollFilter.output());
// zamenjeno pitch i roll zbog orijentacije senzora
	//Serial.print("ypr\t");
	//Serial.print(yawStats.mean());
	//Serial.print("\t");
	//Serial.print(rollStats.mean());
	//Serial.print("\t");
	//Serial.println(pitchStats.mean());
	yawAngle.add(yawStats.mean());
	pitchAngle.add(pitchStats.mean());
	rollAngle.add(rollStats.mean());
	yaw2=yawAngle.read();
	roll2=pitchAngle.read();;
	pitch2=rollAngle.read();;
	
}

void Gyroscope::checkFIFOOverFlow(){
	mpuIntStatus = gyro.getIntStatus();
	fifoCount = gyro.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {  // check for overflow (this should never happen unless our code is too inefficient)
		// reset so we can continue cleanly
		gyro.resetFIFO();
		while (1){
			digitalWrite(RTD, HIGH);
			delay(500);
			digitalWrite(RTD, LOW);
		}
	}
}