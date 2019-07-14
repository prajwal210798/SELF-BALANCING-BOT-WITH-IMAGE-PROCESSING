
void moveMotors(float motorinput){
  if(motorinput>0)
   {
   //Serial.println("forward motion");
   analogWrite(right[1],motorinput);
   analogWrite(right[0],0);
   analogWrite(left[0],0);
   analogWrite(left[1],motorinput);
   }
   else if(motorinput<0)
   {
    motorinput=abs(motorinput);
    //backward motion
    analogWrite(right[0],motorinput);
    analogWrite(right[1],0);
    analogWrite(left[1],0);
    analogWrite(left[0],motorinput);
   }
   
  else if(motorinput>-10 && motorinput<10)
   {//INTEGRALTERM=0;
   //motorstop
    analogWrite(right[0],0);
    analogWrite(right[1],0);
    analogWrite(left[1],0);
    analogWrite(left[0],0);
   }
   

  
  
}
