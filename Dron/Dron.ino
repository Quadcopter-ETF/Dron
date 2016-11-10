#include <SPI.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Gyroscope.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 15;
//Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 1.4;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.05;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 15; //Gain setting for the pitch D-controller.
int pid_max_pitch = 400;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02 ;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.02;               //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct MSG {
  int8_t  Throttle, Yaw, Pitch, Roll;
} msg;

Gyroscope gyro(42);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


RF24 radio(7, 8);
const uint64_t rxAddr = 0xE8E8F0F0E1LL;

#define MAX_Signal 1600
#define MIN_Signal 700
#define MOTOR_PINur 9
#define MOTOR_PINul 10 // dod
#define MOTOR_PINdr 11 // dod 
#define MOTOR_PINdl 12 // dod
Servo motor_ur;
Servo motor_ul;
Servo motor_dr;
Servo motor_dl;
float servoSignal = MIN_Signal;
float yaw, pitch, roll;
int batPin = 8;
float battery_voltage;

float esc_1, esc_2, esc_3, esc_4;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


int start, timepassed, flag;

int MinThrottle = 0;
int MaxThrottle = 100;
int MinRoll = -10;
int MaxRoll = 10;
int MinPitch = -10;
int MaxPitch = 10;
int MinYaw = -30;
int MaxYaw = 30;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  // Gyro
  attachInterrupt(2, dmpDataReady, RISING);
  gyro.bootUp();
  // Radio and Motors
  radio.begin();
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();
  motor_ur.attach(MOTOR_PINur);
  motor_ul.attach(MOTOR_PINul);
  motor_dr.attach(MOTOR_PINdr);
  motor_dl.attach(MOTOR_PINdl);
  motor_ur.writeMicroseconds(MIN_Signal);
  motor_ul.writeMicroseconds(MIN_Signal);
  motor_dr.writeMicroseconds(MIN_Signal);
  motor_dl.writeMicroseconds(MIN_Signal);
  start = millis();
  flag = 0;
  pinMode(42, OUTPUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // if (analogRead(batPin) < 190) digitalWrite(42, HIGH);
  timepassed = millis() - start;
  if (mpuInterrupt) {
    GyroRead(&gyro, mpuInterrupt, yaw, pitch, roll);
    gyro_roll_input = roll;
    gyro_pitch_input = pitch;
    gyro_yaw_input = yaw;
  }

  //Reset the pid controllers for a bumpless start.
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
  // Flag is used to filter out first few readings of Gyro, just for increased precision
  if (timepassed > 3000 && flag == 0) {
    pid_roll_setpoint = 0 ;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;
    flag = 1;
  }
  else if (flag == 1) {
    //PID inputs are known. So we can calculate the pid output.
    if (radio.available()) {
      radio.read(&msg, sizeof(msg));
    }
    else {
      // Start Descending
      int dnow = millis();
      if ((millis() - dnow) > 500) {
        servoSignal -= 50;
        if (servoSignal < MIN_Signal) servoSignal = MIN_Signal;
        dnow = millis();
      }
    }

    msg.Throttle=map(msg.Throttle,0,100,MIN_Signal,MAX_Signal);
    msg.Roll=constrain(msg.Roll,-10,10);
    msg.Pitch=constrain(msg.Pitch,-10,10);
    msg.Yaw=constrain(msg.Yaw,-10,10);
    servoSignal = msg.Throttle;
    pid_roll_setpoint = msg.Roll ;
    pid_pitch_setpoint = msg.Pitch;
    pid_yaw_setpoint = msg.Yaw;
    
    calculate_pid();

    esc_1 = servoSignal - pid_output_pitch - pid_output_roll - pid_output_yaw ;
    esc_2 = servoSignal - pid_output_pitch + pid_output_roll + pid_output_yaw ;
    esc_3 = servoSignal + pid_output_pitch - pid_output_roll + pid_output_yaw ;
    esc_4 = servoSignal + pid_output_pitch + pid_output_roll - pid_output_yaw ;

    esc_1=constrain(esc_1,MIN_Signal,MAX_Signal);
    esc_2=constrain(esc_2,MIN_Signal,MAX_Signal);
    esc_3=constrain(esc_3,MIN_Signal,MAX_Signal);
    esc_4=constrain(esc_4,MIN_Signal,MAX_Signal);
    
    //  Serial.print("PID: ");
    Serial.println("ESC");
    Serial.print("UR:");
    Serial.print(esc_1);
    Serial.print(" ");
    Serial.print("UL:");
    Serial.print(esc_2);
    Serial.print(" ");
    Serial.print("DR:");
    Serial.print(esc_3);
    Serial.print(" ");
    Serial.print("DL:");
    Serial.println(esc_4);
    Serial.println("YPR");
    Serial.print(yaw);
    Serial.print("   ");
    Serial.print(pitch);
    Serial.print("   ");
    Serial.print(roll);
    Serial.println("   ");
    Serial.println("MSG");
    Serial.print(msg.Throttle);
    Serial.print("   ");
    Serial.print(msg.Roll);
    Serial.print("   ");
    Serial.print(msg.Pitch);
    Serial.print("   ");
    Serial.print(msg.Yaw);
    Serial.println("   ");

    motor_ur.writeMicroseconds(esc_1);
    motor_ul.writeMicroseconds(esc_2);
    motor_dr.writeMicroseconds(esc_3);
    motor_dl.writeMicroseconds(esc_4);

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}




