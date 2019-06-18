float Kp_speed = 2;          
float Ki_speed = 0;                  
float Kd_speed = 3;               
float iTermsp; 
 float lastTime_speed = 0;    
 unsigned long nextTime_speed = 0;     
//float lastTime = 0;    
float maxpid_output_speed = 20;    
float minpid_output_speed=-20;
#define STD_LOOP_TIME 20
unsigned long nexttime_speed = 0;
float speed,Current_speed=0,oldCurrent_speed=0,angleinput;
//double target_speed=0.0;
void pid_set_speed(float target_speed,float speed){
  // Only run the controller once the time interval has passed
  //if (nextTime < millis()) {
    //nexttime_speed = millis() + STD_LOOP_TIME;
    //speed = speed_output;
    angleinput = PID_speed(target_speed,speed);
     
    Serial.print("angleinput");
    Serial.println(angleinput);
    pid_set(angleinput,compAngleY);
    status_pid_speed=1;
  //}
}

/****** PID CONTROLLER *****/
float PID_speed(float target, float current) { 
  // Calculate the time since function was last called
  float thisTime_speed = millis();
  float dT = thisTime_speed - lastTime_speed;
  lastTime_speed = thisTime_speed;

  // Calculate error between target and current values
  float error = target - current;
  //float error=0;
  float pids = 0;

  // Calculate the integral and derivative terms
  iTermsp += error * dT; 
  float dTerm = (Current_speed - oldCurrent_speed) / dT;

  // Set old variable to equal new ones
  oldCurrent_speed = Current_speed;

  // Multiply each term by its constant, and add it all up
 // Serial.print("error*Kp_speed");
  //Serial.println(error*Kp_speed);
  //Serial.print("iTerm * Ki_speed");
  //Serial.println(iTerm * Ki_speed);
  //iTerm * Ki_speed
  pids = (error * Kp_speed) + (iTermsp * Ki_speed) - (dTerm * Kd_speed);

  // Limit PID value to maximum values
    Serial.print("pid");
    Serial.println(pids);  
  if ( pids > maxpid_output_speed) pids = maxpid_output_speed;
  else if ( pids <minpid_output_speed) pids = minpid_output_speed;
  //Serial.print("pid");
//Serial.println(pids);  
   return pids;
}

/*  float PID_speed(float target, float current) { 

                                   
     float thisTime_speed = millis();
  float dT = thisTime_speed - lastTime_speed;
  lastTime_speed = thisTime_speed;

  // Calculate error between target and current values
  float error = target - current;
  //float error=0;
  float pids = 0;                                
  // error=SETANGLE-CURRENTANGLE; //proportional term
  // Serial.print("error ");
   //Serial.println(error);
   //ERRORSUM=ERRORSUM+error;
   if(error>-3 && error<3)
   {iTermsp=iTermsp+Ki_speed*error;
    if(iTermsp>155) iTermsp= 155;
      else if(iTermsp< -155) iTermsp= -155;             
                 }
   //DERIVATIVETERM=(error-PREVERROR);
   //DERIVATIVETERM=(CURRENTANGLE-PREVANGLE);
   float dTerm = (Current_speed - oldCurrent_speed) / dT;
    oldCurrent_speed = Current_speed;
   pids=Kp_speed*error+iTermsp-Kd_speed*dterm;
   
   //PREVERROR=error;
   //PREVANGLE=CURRENTANGLE;

                          //*** ANGLE PID COMPLETE***

   Serial.print("pid");
   Serial.println(pids);  
  if ( pids > maxpid_output_speed) pids = maxpid_output_speed;
  else if ( pids <minpid_output_speed) pids = minpid_output_speed;

  }*/








