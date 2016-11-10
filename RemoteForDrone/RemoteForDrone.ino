#include "MovingAVG.h"
#include  <SPI.h> //Include SPI Code Library which can be downloaded below
#include "nRF24L01.h" //Include NRF24L01 Code Library which can be downloaded below
#include "RF24.h" //Inlcude NRF24 Code Library which can be downloaded below

struct MSG { 
  int8_t  Throttle,Yaw, Pitch, Roll;
} msg;

int TurnOnOffPin = 3;
int JX1 = A0;
int JY1 = A1;
int JX2 = A2;
int JY2 = A3;
MovingAvgInt rX1;
MovingAvgInt rY1;
MovingAvgInt rX2;
MovingAvgInt rY2;
int MinThrottle = 0;
int MaxThrottle = 100;
int MinRoll = -10;
int MaxRoll = 10;
int MinPitch = -10;
int MaxPitch = 10;
int MinYaw = -30;
int MaxYaw = 30;
RF24 radio(7, 8); // The NRF24L01 Pin CE and Pin CSN
const uint64_t pipe = 0xE8E8F0F0E1LL; //Communication Pip Address
int Throttle, Roll, Pitch, Yaw;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe); //Open Communication Pipe
 // radio.setChannel(120);
  radio.stopListening();
  pinMode(TurnOnOffPin, OUTPUT);
}

void loop() {
  rX1.add(analogRead(JX1));
  rY1.add(analogRead(JY1));
  rX2.add(analogRead(JX2));
  rY2.add(analogRead(JY2));

  if (rX1.read() >= 510) {
    Throttle = map(rX1.read(), 510, 1024, MinThrottle, MaxThrottle);
  }
  else Throttle = MinThrottle;
  msg.Throttle = Throttle;

  if (rY2.read() >= 520 || rY2.read() <= 490) {
    Roll = map(rY2.read(), 0, 1024, MinRoll, MaxRoll);
  }
  else Roll = 0;
  msg.Roll = Roll;

  if (rX2.read() >= 520) {
    Pitch = map(rX2.read(), 520, 1024, 0, MaxPitch);
  }
  else if (rX2.read() < 490) {
    Pitch = map(rX2.read(), 0, 490 , MinPitch, 0);
  }
  else Pitch = 0;
  msg.Pitch = Pitch;

  if (rY1.read() >= 600) {
    Yaw = map(rY1.read(), 600, 1024, 0, MaxYaw);
  }
  else if ( rY1.read() < 400) {
    Yaw = map(rY1.read(), 0, 400, MinYaw, 0);
  }
  else Yaw = 0;
  msg.Yaw = Yaw;

  if (digitalRead(TurnOnOffPin)) {
    msg.Throttle = 0;
  }
  Serial.print(msg.Throttle);
  Serial.print(" ");
  Serial.print(msg.Roll);
  Serial.print(" ");
  Serial.print(msg.Pitch);
  Serial.print(" ");
  Serial.print(msg.Yaw);
  Serial.println(" ");
  radio.write(&msg, sizeof(msg));
  

}
