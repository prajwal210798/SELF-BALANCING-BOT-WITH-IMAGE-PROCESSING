
#include<Wire.h>
//#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro(0x68);
#include<math.h>
#include<helper_3dmath.h>
VectorInt16  gyroReadout;
VectorInt16  aaReadout;


const int MPU_addr=0x68; // I2C address of the MPU-6050
  

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication



uint32_t timer;
 
double compAngleX, compAngleY;

  

  


                                  //**motor 
int right[2]={10,9};
int left[2]={3,6};

  //filter//
int acc_gyro_error;
VectorFloat Gyro_raw_error,acc_raw_error,gyroReadoutf,aaReadoutf;
double filtered_pitch,prev_filtered_acc_signal, gyro_prev_filtered_Signal,filtered_gyroangle,prev_filtered_pitch_signal;
double gyroangle,dt,prev_unfiltered_gyro;

double delta=0.00114;
uint8_t i2cData[14];


                                 //functions for i2c communication with mpu6050//
 uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  Wire.endTransmission(true);
  return 0; // Success
}
  
  
  
  
  
  void setup(){

  


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
  accelgyro.setXAccelOffset(-1320);
  accelgyro.setYAccelOffset(1385);
  accelgyro.setZAccelOffset(1109);
  accelgyro.setXGyroOffset(-68);
  accelgyro.setYGyroOffset(52);
  accelgyro.setZGyroOffset(-36);

  //accelgyro.setXAccelOffset(ax_offset);
 //accelgyro.setYAccelOffset(ay_offset);
 //accelgyro.setZAccelOffset(az_offset);
 //accelgyro.setXGyroOffset(gx_offset);
 //accelgyro.setYGyroOffset(gy_offset);
 //accelgyro.setZGyroOffset(g_offset);

  

  timer = micros();

  }
void loop(){

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  if(dt>=0.019) //almost 0.02 sec 
  {//Serial.println(micros()-timer);
   timer = micros();
  

  
  accelgyro.getMotion6(&aaReadout.x,&aaReadout.y,&aaReadout.z,&gyroReadout.x,&gyroReadout.y,&gyroReadout.z); 

       
  double pitch = atan(-aaReadout.x/ sqrt(pow(aaReadout.y,2)+ pow(aaReadout.z,2))) * RAD_TO_DEG;
 
 
 double gyroYrate = gyroReadout.y/ 131.0;


  
 // compAngleY = 0.4875*(compAngleY+gyroYrate*dtg) + 0.5125*pitch;   //4hz
 //compAngleY = 0.975*(compAngleY+gyroYrate*dtg) + 0.025*pitch;      //2hz
    
    compAngleY = 0.9875*(compAngleY+gyroYrate*dt) + 0.012410*pitch;   //cutt of frequency=1hz ;AND SAMPLING time =0.02sec ;
    //these numbers may be different(ie different cutt off frequency and different sampling time and 
    //need to change for  different conditions
    //a pdf is available in my github account to discuss to how to find these numbers.
  
  Serial.print("compAngleY");Serial.print("\t");Serial.println(compAngleY);
  pid_set(-2.6,compAngleY,dt);   //-2.6 is desired equilibrium angle for my bot may be different in your case
}
}
