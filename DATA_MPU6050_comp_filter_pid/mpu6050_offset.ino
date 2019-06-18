
void calcoffset(){
  state=0;
  while(state!=3)
  {
if (state==0){
Serial.println("\nReading sensors for first time...");
meansensors();
state++;
delay(1000);
}
if (state==1) {
Serial.println("\nCalculating offsets...");
calibration();
state++;
delay(1000);
}
if (state==2) {
meansensors();
Serial.println("\nFINISHED!");
Serial.print("\nSensor readings with offsets:\t");
Serial.print(mean_ax); 
Serial.print("\t");
Serial.print(mean_ay); 
Serial.print("\t");
Serial.print(mean_az); 
Serial.print("\t");
Serial.print(mean_gx); 
Serial.print("\t");
Serial.print(mean_gy); 
Serial.print("\t");
Serial.println(mean_gz);
Serial.print("Your offsets:\t");
Serial.print(ax_offset); 
Serial.print("\t");
Serial.print(ay_offset); 
Serial.print("\t");
Serial.print(az_offset); 
Serial.print("\t");
Serial.print(gx_offset); 
Serial.print("\t");
Serial.print(gy_offset); 
Serial.print("\t");
Serial.println(gz_offset); 
Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
state++;
}
}
}

void meansensors(){
long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
while (i<(buffersize+101)){
// read raw accel/gyro measurements from device
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
buff_ax=buff_ax+ax;
buff_ay=buff_ay+ay;
buff_az=buff_az+az;
buff_gx=buff_gx+gx;
buff_gy=buff_gy+gy;
buff_gz=buff_gz+gz;
}
if (i==(buffersize+100)){
mean_ax=buff_ax/buffersize;
mean_ay=buff_ay/buffersize;
mean_az=buff_az/buffersize;
mean_gx=buff_gx/buffersize;
mean_gy=buff_gy/buffersize;
mean_gz=buff_gz/buffersize;
}
i++;
delay(2); //Needed so we don't get repeated measures
}
}
void calibration(){
ax_offset=-mean_ax/8;
ay_offset=-mean_ay/8;
az_offset=(16384-mean_az)/8;
gx_offset=-mean_gx/4;
gy_offset=-mean_gy/4;
gz_offset=-mean_gz/4;
while (1){
int ready=0;
 //unsigned char datav[6] = {0, 0, 0, 0};
 //datav[0] = (gx_offset >> 8) & 0xff;
 //datav[1] = (gx_offset) & 0xff;
 //datav[2] = (gy_offset >> 8) & 0xff;
 //datav[3] = (gy_offset) & 0xff;
 //datav[4] = (gz_offset >> 8) & 0xff;
 //datav[5] = (gz_offset) & 0xff;
 //i2cWrite( 0x13, 2, &datav[0]);
 //i2cWrite(0x15, 2, &datav[2]);
 //i2cWrite(0x17, 2, &datav[4]);

//accset();
//Serial.println(ax_offset);
accelgyro.setXAccelOffset(ax_offset);
accelgyro.setYAccelOffset(ay_offset);
accelgyro.setZAccelOffset(az_offset);
accelgyro.setXGyroOffset(gx_offset);
accelgyro.setYGyroOffset(gy_offset);
accelgyro.setZGyroOffset(gz_offset);
  //Serial.println(gx_offset)
 //i2cData[0] = (gx_offset >> 8) & 0xff; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
 //i2cData[1] = (gx_offset) & 0xff; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
 //i2cData[2] = (gy_offset>>8) & 0xff; // Set Gyro Full Scale Range to ±250deg/s
 //i2cData[3] =  (gy_offset) & 0xff;// Set Accelerometer Full Scale Range to ±2g
  //while (i2cWrite(0x13, i2cData, 4, true)); // Write to all four registers at once
 //gyroset();
 //i2cDatacc[0] = (ax_offset >> 8) & 0xff; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
 //i2cDatacc[1] = (ax_offset) & 0xff; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
 //i2cDatacc[2] = (ay_offset>>8) & 0xff; // Set Gyro Full Scale Range to ±250deg/s
 //i2cDatacc[3] = (ay_offset) & 0xff;// Set Accelerometer Full Scale Range to ±2g
 //i2cDatacc[4]=  (az_offset>>8) & 0xff;
 //i2cDatacc[5]=  (az_offset) & 0xff;
 //while(i2cWrite(0x77, i2cDatacc, 4, true));
 //Serial.println(((i2cDatacc[0] << 8) | i2cDatacc[1]));
 
 
 
 
// i2cData[]=0;
 
  //while (i2cRead(0x13, i2cData, 4));
  //accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  //accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  //accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  //gxx = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  //gyy = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  //while (i2cRead(0x77, i2cDatacc, 4));
  //accX = (int16_t)((i2cDatacc[0] << 8) | i2cDatacc[1]);
  //accY = (int16_t)((i2cDatacc[2] << 8) | i2cDatacc[3]);
  //accZ = (int16_t)((i2cDatacc[4] << 8) | i2cDatacc[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  //gxx = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  //gyy = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  //gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
//datav[0]=i2c.read(0x68,0x13,1);
//datav[1]=i2c.read(0x68,0x13,1)
//datav[2]= i2c.read(0x68,0x15,1)
//datav[3]= i2c.read(0x68,0x16,1)
//if (i2cRead(0x17,&data[2],4)
//accel_bias[0] = ((long)data[0]<<8) | data[1];
//accel_bias[1] = ((long)data[2]<<8) | data[3];
//accel_bias[2] = ((long)data[4]<<8) | data[5];
//Serial.println(gxx);
//Serial.println(gyy);
//Serial.println(ax_offset);
meansensors();
Serial.println("...");
if (abs(mean_ax)<=acel_deadzone) ready++;
else ax_offset=ax_offset-mean_ax/acel_deadzone;
if (abs(mean_ay)<=acel_deadzone) ready++;
else ay_offset=ay_offset-mean_ay/acel_deadzone;
if (abs(16384-mean_az)<=acel_deadzone) ready++;
else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
if (abs(mean_gx)<=giro_deadzone) ready++;
else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
if (abs(mean_gy)<=giro_deadzone) ready++;
else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

if (ready==5) break;
}

}




void gyroset()
{i2cData[0] = (gx_offset >> 8) & 0xff; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
 i2cData[1] = (gx_offset) & 0xff; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
 i2cData[2] = (gy_offset>>8) & 0xff; // Set Gyro Full Scale Range to ±250deg/s
 i2cData[3] =  (gy_offset) & 0xff;// Set Accelerometer Full Scale Range to ±2g
 while (i2cWrite(0x13, i2cData, 4, true)); // Write to all four registers at once

}

void accset()
{i2cDatacc[0] = (ax_offset >> 8) & 0xff; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
 i2cDatacc[1] = (ay_offset) & 0xff; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
 i2cDatacc[2] = (ay_offset>>8) & 0xff; // Set Gyro Full Scale Range to ±250deg/s
 i2cDatacc[3] = (ay_offset) & 0xff;// Set Accelerometer Full Scale Range to ±2g
 //i2cDatacc[4]=  (az_offset>>8) & 0xff;
 //i2cDatacc[5]=  (az_offset) & 0xff;
 while (i2cWrite(0x77, i2cDatacc, 4, true)); // Write to all four registers at once

}




