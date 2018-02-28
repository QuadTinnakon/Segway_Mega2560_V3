/*
project_balance robot arduino mega 2560_RMUTSB 
Balancing Robot arduino. ,,A two-wheel inverted pendulum robot
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

date: 22-02-2561(2018)  V.1 Segway_Mega2560_V1  ,, Read mpu6050 ,,X Y Z
date: 26-02-2561(2018)  Segway_Mega2560_V2      ,, Roop time 180 Hz  ,PID Control ,Wheel Encoder
date: 27-02-2561(2018)  Segway_Mega2560_V3      ,,Potentiometer attached to analog input 0,1,2,3 gain kp,ki,kd

support:  Board arduino mega 2560 ,, GY521
• ATmega2560  16 MHz crystal oscillator
• MPU6050 Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• L298N motor driver 2A 7-12 v
• Gear Motor 3V - 12V , 1:48
• Optical Wheel Encoder 15mA 3.3V-5V LM393 comparator

---------motor---------
int MOTOR_FrontL_PIN = 9;
int MOTOR_FrontR_PIN = 10;
int IN1A_PIN = 22;
int IN2A_PIN = 23;
int IN3B_PIN = 24;
int IN4B_PIN = 25;

*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "mpu6050.h"
#include "ahrs_tin.h"
#include "Control_PID.h"
#include "motorX4.h"

void setup()
{
  Serial.begin(57600);//38400
  Serial.print("Segway_Mega2560_V3_180Hz");Serial.println("\t");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //Serial1.begin(115200);//CRIUS Bluetooth Module pin code 0000
  //Serial3.begin(38400);//3DR Radio Telemetry Kit 433Mhz
  motor_initialize();//find motor.h
  pinMode(EnCR_PIN, INPUT);
  pinMode(EnCL_PIN, INPUT);
  attachInterrupt(0, ISRe, CHANGE);
  attachInterrupt(1, ISLe, CHANGE);
  Wire.begin();
  delay(1);
  mpu6050_initialize();
  delay(1); 
  digitalWrite(13, HIGH);
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 
  delay(1);
      for(uint8_t i=0; i<50; i++) 
    {
     mpu6050_Gyro_Values();
     mpu6050_Accel_Values();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  ahrs_initialize();//ahrs.h
  Serial.print("Segway_Mega2560_V3_180Hz");Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop()
{
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 3 samples = 2760 us  && gyroSamples < 5  && gyroSamples < 5
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 10001; 
   }   
    if (Dt_roop >= 5000)//5560 = 179 Hz 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Get_accel();
      mpu6050_Get_gyro();
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
////////////////Low pass filter/////////////////////////////////
      AccXf = AccX;
      AccYf = AccY;
      AccZf = AccZ;
      AccXf = AccXf + (AccX - AccXf)*15.6*G_Dt;//29.6 15.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      AccYf = AccYf + (AccY - AccYf)*15.6*G_Dt;//15.4
      AccZf = AccZf + (AccZ - AccZf)*15.6*G_Dt;//15.4
//////////////////////////////////////////////////////////
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, 5, 0, 0, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
      //ahrs_updateMARG(GyroXf, GyroYf, GyroZf, filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], 5, 0, 0, G_Dt);
      //x_angle = x_angle + (GyroXf*RAD_TO_DEG*G_Dt);
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);
      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PID Control///////////
     Control_PIDRate();//Control_PID.h
//////Out motor///////////
//armed = 1;
     motor_Mix();//"motor.h"
/////////////////////////
     motor_command(); 
////////end Out motor//////

         if (frameCounter % TASK_5HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            Serial.print(millis()/1000.0);Serial.print("\t");
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZf);Serial.print("\t");  
            //Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            
            //Serial.print(DCM10);Serial.print("\t");
            //Serial.print(DCM11);Serial.print("\t");
            //Serial.print(DCM12);Serial.print("\t");
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ);Serial.print("\t");
            //Serial.print(AccZf2,3);Serial.print("\t");
            //Serial.print(AccZf3,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");

            
            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(courseRads);Serial.print("\t"); 
            //Serial.print(ahrs_r);Serial.print("\t");
            Serial.print(ahrs_p);Serial.print("\t");  
            //Serial.print(ahrs_y);Serial.print("\t");  
            //Serial.print(Heading);Serial.print("\t");
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(u_roll);Serial.print("\t");
            //Serial.print(u_pitch);Serial.print("\t");
            //Serial.print(u_yaw);Serial.print("\t");
            
            //Serial.print(pitch_I_rate);Serial.print("\t");
            //Serial.print(yaw_I_rate);Serial.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(L_encoder);Serial.print("\t");
            //Serial.print(R_encoder);Serial.print("\t");
            
            //Serial.print(sensorValue1);Serial.print("\t");
            //Serial.print(sensorValue2);Serial.print("\t");
            //Serial.print(sensorValue3);Serial.print("\t");
            //Serial.print(sensorValue4);Serial.print("\t");
            Serial.print(Kp_ratePitch);Serial.print("\t");
            Serial.print(Ki_ratePitch);Serial.print("\t");
            Serial.print(Kd_ratePitch);Serial.print("\t");
            Serial.print(Kp_levelPitch);Serial.print("\t");
            
            Serial.print(motor_FrontL);Serial.print("\t");
            Serial.print(motor_FrontR);Serial.print("\t");

            //Serial.print(gyroSamples2);Serial.print("\t");
            //Serial.print(1/G_Dt);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            sensorValue1 = analogRead(A0);//523 ,,kp
            sensorValue2 = analogRead(A1);//551 ,,ki
            sensorValue3 = analogRead(A2);//524 ,,kd
            sensorValue4 = analogRead(A3);//560 ,,kpp
            Kp_ratePitch = sensorValue1*1.62/523.0;//0.42
            Ki_ratePitch = sensorValue2*0.62/551.0;//0.125
            Kd_ratePitch = sensorValue3*0.0445/524.0;//0.0145
            Kp_levelPitch = sensorValue4*63.2/560.0;//23.2
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
void ISLe() {
   if(motorLdri == 1){
    L_encoder++;
   }
   else{
     L_encoder--;
   }
}
void ISRe() {
   if(motorRdri == 1){
    R_encoder++;
   }
   else{
     R_encoder--;
   }
}
