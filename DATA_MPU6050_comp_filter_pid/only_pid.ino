float Kp_angle = 7;          
float Ki_angle = 0;                  
float Kd_angle = 3;               
float iTermangle;           
float lastTime_angle = 0;    
float maxPID_angle = 150;    
float minPID_angle=-150;
//float angleoutput;
#define STD_LOOP_TIME 10
unsigned long nextTime_angle = 0;
float old_angle_target,Current_angle,oldCurrent_angle,angle,angleoutput;

void pid_set(float target,float angle){
  // Only run the controller once the time interval has passed
     old_angle_target=target;
    angleoutput = PID_angle(target, angle);
    moveMotors(angleoutput);
    Serial.println(angleoutput);

  }
//}
void pid_set(float angle)
 {
  angleoutput = PID_angle(old_angle_target,angle);
  moveMotors(angleoutput);
   Serial.println(angleoutput);

}

/****** ANGLE PID CONTROLLER *****/
float PID_angle(float target, float current) { 
  // Calculate the time since function was last called
  float thisTime_angle = millis();
  float dT = thisTime_angle - lastTime_angle;
  lastTime_angle = thisTime_angle;

  // Calculate error between target and current values
  float error = target - current;
  float pidangle = 0;

  // Calculate the integral and derivative terms
  iTermangle += error * dT; 
  float dTerm = (Current_angle - oldCurrent_angle) / dT;

  // Set old variable to equal new ones
  oldCurrent_angle = Current_angle;

  // Multiply each term by its constant, and add it all up
  pidangle = (error * Kp_angle) + (iTermangle * Ki_angle) - (dTerm * Kd_angle);

  // Limit PID value to maximum values
  if ( pidangle > maxPID_angle) pidangle = maxPID_angle;
  else if ( pidangle < -maxPID_angle) pidangle = minPID_angle;
   return pidangle;
}
