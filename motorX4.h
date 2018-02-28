/*
project_balance robot arduino mega 2560_RMUTSB  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
//#include "motorX4.h"
int MOTOR_FrontL_PIN = 9;
int MOTOR_FrontR_PIN = 10;
int IN1A_PIN = 22;
int IN2A_PIN = 23;
int IN3B_PIN = 24;
int IN4B_PIN = 25;

int EnCL_PIN = 3;//IN1
int EnCR_PIN = 2;//IN0
int motorLdri = 0;
int motorRdri = 0;

float motor_FrontL = 0.0;
float motor_FrontR = 0.0;

int motor_FrontLf = 0;
int motor_FrontRf = 0;

void motor_initialize() 
{
  pinMode(MOTOR_FrontL_PIN,OUTPUT);  
  pinMode(MOTOR_FrontR_PIN,OUTPUT); 
  pinMode(IN1A_PIN,OUTPUT);  
  pinMode(IN2A_PIN,OUTPUT);
  pinMode(IN3B_PIN,OUTPUT);  
  pinMode(IN4B_PIN,OUTPUT);
  digitalWrite(IN1A_PIN, LOW);
  digitalWrite(IN2A_PIN, LOW);
  digitalWrite(IN3B_PIN, LOW);
  digitalWrite(IN4B_PIN, LOW);
  analogWrite(MOTOR_FrontL_PIN, 0);
  analogWrite(MOTOR_FrontR_PIN, 0);

  delay(10);
// pin 9 pin 10 timer 2 
     TCCR2B = TCCR0B  & 0b11111000 | 0x03;//0x01  31372.55 Hz
     TCCR2B = TCCR2B  & 0b11111000 | 0x03;//0x02  3921.16  Hz
  analogWrite(MOTOR_FrontL_PIN, 1);
  analogWrite(MOTOR_FrontR_PIN, 1);   
}

//motor command
void motor_command() 
{
  //////////motor 1 pin 9//////////////////////////////
  if(motor_FrontLf > 0){//Motor 1 Forward
   analogWrite(MOTOR_FrontL_PIN, (motor_FrontLf));// Offset +- 10
   digitalWrite(IN1A_PIN, HIGH);
   digitalWrite(IN2A_PIN, LOW);
   motorLdri = 1;
  }
  else{//Motor 1 Reverse
   analogWrite(MOTOR_FrontL_PIN, (motor_FrontLf*-1));
   digitalWrite(IN1A_PIN, LOW);
   digitalWrite(IN2A_PIN, HIGH);
   motorLdri = 0;
  }
///////motor 2 pin 10//////////////////////////////////////////
  if(motor_FrontRf > 0){//Motor 2 Forward
   analogWrite(MOTOR_FrontR_PIN, (motor_FrontRf));
   digitalWrite(IN3B_PIN, HIGH);
   digitalWrite(IN4B_PIN, LOW);
   motorRdri = 0;
  }
  else{//Motor 2 Reverse
   analogWrite(MOTOR_FrontR_PIN, (motor_FrontRf*-1));
   digitalWrite(IN3B_PIN, LOW);
   digitalWrite(IN4B_PIN, HIGH);
   motorRdri = 1;
  }
///////////////////////////////////////////////////////
}

void motor_Mix(){
      motor_FrontL = u_yaw + u_pitch;//Front L 
      motor_FrontR = u_yaw - u_pitch;//Front R
      motor_FrontLf = constrain(motor_FrontL*1.3, -255, 255);//motor slow 1.3
      motor_FrontRf = constrain(motor_FrontR, -255, 255);
}
