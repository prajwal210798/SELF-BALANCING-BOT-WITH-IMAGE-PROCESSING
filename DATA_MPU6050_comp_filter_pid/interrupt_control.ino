
void leftEncoder() { 
  if(PIND & B0001000) // read pin 4
    leftCounter--;
  else
    leftCounter++;    
}
//void rightEncoder() {
  //if(PIND & _BV(PIND5)) // read pin 5
    //rightCounter--;
  //else
    //rightCounter++;  
//}
long readEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
//long readRightEncoder() {
  //return rightCounter;
//}
