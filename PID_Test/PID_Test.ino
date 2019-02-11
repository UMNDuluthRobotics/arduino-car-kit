#include <SoftwareSerial.h>
#include <Wire.h>
int pinLB = 8;                  // pretty sure correct
int pinLF = 7;                  //pin of controlling diversion----IN2 of motor driver board
int pinRB = 2;                  //pin of controlling diversion----IN3 of motor driver board
int pinRF = 4;                  //pin of controlling diversion----IN4 of motor driver board
unsigned char Lpwm_val = 180;   //the speed of left wheel at 180 in initialization
unsigned char Rpwm_val = 180;   //the speed of right wheel at 180 in initialization
#define Lpwm_pin  5             //pin of controlling speed---- ENA of motor driver board CORRECT
#define Rpwm_pin  6           //pin of controlling speed---- ENB of motor driver board

void M_Control_IO_config(void)//initialized function of IO of motor driver
{
  pinMode(pinLB, OUTPUT); // pin 2--IN1 of motor driver board
  pinMode(pinLF, OUTPUT); // pin 4--IN2 of motor driver board
  pinMode(pinRB, OUTPUT); // pin 7--IN3 of motor driver board
  pinMode(pinRF, OUTPUT); // pin 8--IN4 of motor driver board
  pinMode(Lpwm_pin, OUTPUT); // pin 5  (PWM) --ENA of motor driver board
  pinMode(Rpwm_pin, OUTPUT); // pin 10 (PWM) --ENB of motor driver board
}
void Set_Speed(unsigned char Left, unsigned char Right) //setting function of speed
{
  analogWrite(Lpwm_pin, Left);
  analogWrite(Rpwm_pin, Right);
}
void advance()     // going forwards
{
  digitalWrite(pinRB, LOW); // making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); //  making motor move towards left rear
  digitalWrite(pinLF, HIGH);
}
void turnR()        //turning on the right(dual wheels)
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH); //making motor move towards left front
}
void turnL()        //turning on the left(dual wheels)
{
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH );  //making motor move towards right front
  digitalWrite(pinLB, HIGH);  //making motor move towards left rear
  digitalWrite(pinLF, LOW);
}
void stopp()         //stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
}
void back()          //back
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); //making motor move towards left rear
  digitalWrite(pinLF, LOW);
}
void setup()
{
  M_Control_IO_config();        //motor controlling the initialization of IO
  Set_Speed(Lpwm_val, Rpwm_val); //setting initialization of speed
  stopp();
  Serial.begin(57600);
}

void loop() { // run over and over
  int data = 0;
  if (Serial.available() > 0) {
    data = Serial.read();
    Serial.println(data);
    switch (data) {
      case (119):
        advance();
        break;
      case (97):
        turnL();
        break;
      case (100):
        turnR();
        break;
      case (115):
        back();
        break;
      default:
        stopp();
        break;
    }
  }
}
