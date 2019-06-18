
#include<Wire.h>
//#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro(0x68);


uint8_t i2cDatagyro[4];
uint8_t i2cDatacc[4];
const int MPU_addr=0x68; // I2C address of the MPU-6050
int loopCounter=0;  
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint32_t timer;
double gyroXangle, gyroYangle; 
double compAngleX, compAngleY;
uint8_t i2cData[14];

  
 // void timekeeper() {
// Calculate time since loop began
//float timeChange = millis() - loopStartTime;
// If the required loop time has not been reached, please wait!
//if (timeChange < STD_LOOP_TIME) {
//delay(STD_LOOP_TIME - timeChange);
//} 
// Update loop timer variables
//loopStartTime = millis();   
//}
  
                                           //**filter cofficient**// 
float alphalp = 0.0909,alphahp=0.8641;

float raw_acc_Signal[100]; 
float raw_gyro_Signal[100];
                                        //**filter cofficient done**//




                      //**interrupt initialization**//
 float wheelPosition,wheelVelocity=0,lastWheelPosition,wheel_diameter=.065,time_sampling=1,MAINtarget_speed=6.0,status_pid_speed=0;
 volatile int leftCounter;      


                                  //**motor 
int right[2]={9,10};
int left[2]={5,6};
 
  void setup(){
  //interrupt intialization
  attachInterrupt(0,leftEncoder,RISING); // pin 2
  //attachInterrupt(1,rightEncoder,RISING); // pin 3
  


//motor_intialization

for(int i=0;i<=1;i++)
{
  pinMode(left[i],OUTPUT);
  pinMode(right[i],OUTPUT);

}

pinMode(4,INPUT); //for interrupt_control

//****************//
  Serial.begin(115200);
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register tells  the address of mpu6050 
    Serial.print(F("Error reading sensor"));
    while (1);
  }

 delay(100); // Wait for sensor to stabilize
  accelgyro.setXAccelOffset(-1028);
  accelgyro.setYAccelOffset(1376);
  accelgyro.setZAccelOffset(1147);
  accelgyro.setXGyroOffset(-65);
  accelgyro.setYGyroOffset(41);
  accelgyro.setZGyroOffset(-35);
  //calcoffset();
  //accelgyro.setXAccelOffset(ax_offset);
 //accelgyro.setYAccelOffset(ay_offset);
 //accelgyro.setZAccelOffset(az_offset);
 //accelgyro.setXGyroOffset(gx_offset);
 //accelgyro.setYGyroOffset(gy_offset);
 //accelgyro.setZGyroOffset(g_offset);
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  timer = micros();

  
}
void loop(){
  while (i2cRead(0x3B, i2cData, 12));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  //gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  double dtmain = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  Serial.print("timer in micros sec");
  Serial.println(dtmain*1000000);
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  compAngleX = roll; 
  compAngleY = pitch;
  gyroXangle += gyroXrate * dtmain; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dtmain;
  compAngleX = 0.99 * (compAngleX + gyroXrate * dtmain) + 0.01 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.98 * (compAngleY + gyroYrate * dtmain) + 0.02 * pitch;
 
  loopCounter++;
  //if (loopCounter == 5) {
    Serial.println("in loop");
    loopCounter = 0; // Reset loop counter
    wheelPosition = readEncoder();
    wheelVelocity = (wheelPosition - lastWheelPosition)*3.14*wheel_diameter/dtmain;
    lastWheelPosition = wheelPosition;
    pid_set_speed(MAINtarget_speed,0);
     //}
    //if(status_pid_speed==0)
    //{ Serial.println("in other loop");
   //pid_set(compAngleY);     
   // }status_pid_speed=0;
      
 // Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.println(0);
  Serial.print(compAngleX); Serial.print("\t");


  Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.println("\t");

 
  //delay(333);
}







//void filter_acc_data(float *raw_acc_Signal)
//{//float cleanSignal[100]=0;
 //cleanSignal[1]=rawSignal[1]
//  for(int i=1,i<100,i++)
//oldCleanSignal = cleanSignal;
                 //olddata=raw_acc_Signal[i-1]
//raw_acc_Signal[i] = alphalp * raw_acc_Signal[i] + (1 - alphalp) *raw_acc_Signal[i-1];
//}


//void filter_gyro_data(float *raw_gyro_Signal)
//{float old_raw_data=raw_gyro_Signal[0];
 //cleanSignal[1]=rawSignal[1]
 // for(int i=1,i<100,i++)
//oldCleanSignal = cleanSignal;
                 //olddata=raw_gyro_Signal[i]
//raw_gyro_Signal[i] = alphahp * raw_gyro_Signal[i-1) + alphahp *(raw_gyro_signal[i]-old_raw_data);
//old_raw_data=olddata;
//}

