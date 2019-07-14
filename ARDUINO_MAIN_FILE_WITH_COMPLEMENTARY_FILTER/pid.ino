float Kp_angle = 19.55;          
float Ki_angle = 20;        //these three numbers is very crucial your bot performance need to be tuned           
float Kd_angle = 0.204;              
float iTermangle;           
float lastTime_angle = 0;    
float maxPID_angle = 150;    
float minPID_angle=-150;
float preverror,angleoutput;

//float angleoutput;
unsigned long nextTime_angle = 0;

void pid_set(float target,float angle ,float dt){
  // Only run the controller once the time interval has passed
   
    angleoutput = PID_angle(target, angle , dt);
    moveMotors(angleoutput);
   // Serial.println(angleoutput);

  }


/****** ANGLE PID CONTROLLER *****/
float PID_angle(float target, float current,float dT) { 

   float error=target - current;

  float pidangle = 0;

  iTermangle += error * dT; 
  float dTerm = (error - preverror) / dT;
  preverror=error;

  // Multiply each term by its constant, and add it all up
  pidangle = (error * Kp_angle) + (iTermangle * Ki_angle) - (dTerm * Kd_angle);

  // Limit PID value to maximum values
  if ( pidangle > maxPID_angle) pidangle = maxPID_angle;
  else if ( pidangle < -maxPID_angle) pidangle = minPID_angle;
   return pidangle;
}
