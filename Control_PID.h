/*
project_balance robot arduino mega 2560_RMUTSB   

by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
float u_pitch = 0;
float u_yaw = 0;

float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;

float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

void Control_PIDRate(){
//P-PID By tinnakon
////// PITCH CONTROL  P-PID///////////
  float error_pitch = 0.0 - ahrs_p;//ahrs_p*RAD_TO_DEG
  float error_rate_pitch = 0.0  - GyroYf*RAD_TO_DEG;//pitch-dot
  pitch_I_rate += error_pitch*Ki_ratePitch*G_Dt;//integral 
  pitch_I_rate = constrain(pitch_I_rate, -100, 100);//+-150
  //pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));//derivative 
  pitch_D_rate = (error_rate_pitch - error_rate_pitchold)/G_Dt;//derivative ,,pitch-dot-dot
  error_rate_pitchold = error_rate_pitch;
  u_pitch = error_pitch*Kp_levelPitch + Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate;//PID
  u_pitch = constrain(u_pitch, -255, 255);//+-255
////// YAW CONTROL PID///////////
  float error_rate_yaw = 0.0 - GyroZf*RAD_TO_DEG;
  yaw_I_rate += error_rate_yaw*Ki_rateYaw*G_Dt;//integral 
  yaw_I_rate = constrain(yaw_I_rate, -50, 50);//+-100
  //yaw_D_rate = (0.025*yaw_D_rate/(0.025+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(0.025+G_Dt));//diff and filter
  yaw_D_rate = (error_rate_yaw-error_rate_yawold)/G_Dt;
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate;
  u_yaw = constrain(u_yaw, -100, 100);//+-100
//////////////////////////////////////////////////////////////////////////////////////////////////  
}

![](https://user-images.githubusercontent.com/9403558/36770109-e5ec3c84-1c7a-11e8-9d68-48ae0f53ca2f.jpg)
