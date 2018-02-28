/*
project_balance robot arduino mega 2560_RMUTSB 
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
///////////////Mode///////////////////////////
int Mode = 0;

//////////////////////////////////////////////////////////////////////
//PID--------------Stable
float Kp_X = 0.75;//1.2

float Kp_levelPitch = 73.2;//76.8 23.2 18.2

//P-PID-------------Rate
float Kp_ratePitch = 0.97;//0.97 0.42
float Ki_ratePitch = 0.85;//0.83 2.42
float Kd_ratePitch = 0.031;//0.03 0.0145 0.0345

float Kp_rateYaw = 0.35;//0.35
float Ki_rateYaw = 0.92;//1.12
float Kd_rateYaw = 0.0035;//0.015

//Parameter system
#define m_segway 0.405 //kg 0.405
#define L_segway 0.04 //40 mm
float I_x = 0.0004389;
float I_y = 0.0004389;//I = T^2 *m*g*L / 4pi^2
float I_z = 0.0004389;
float Nm_to_PWM = 47.17;//59.68 , 47.74
////////////////////////////////////////////////////////////////////
//Accelerometer calibration constants; use the Calibrate example from
int A_X_MIN = -4265;    //
int A_X_MAX = 3978;     //
int A_Y_MIN = -4048;    //
int A_Y_MAX = 4113;     //
int A_Z_MIN = -4105;    //
int A_Z_MAX = 4211;     //

////////////////////////////////////////////////////////////////////
#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 36
#define TASK_2HZ 50
#define TASK_1HZ 180
#define RAD_TO_DEG 57.295779513082320876798154814105

//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0;

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;

uint8_t frameCounter = 0;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;

//Encoder
int L_encoder = 0;
int R_encoder = 0.0;
int L_entime = 0;
unsigned long L_prtime = 0;
int R_entime = 0;
unsigned long R_prtime = 0;

//Analog Input
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
