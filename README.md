# SELF-BALANCING-BOT-WITH-IMAGE-PROCESSING
This project is based on how to build self balancing autonomous bot which will be moving according the processed real time videos capturing from the camera module.

I am using</n> 
a>AVR 8050 microprocessor
b>Raspberry pi 3b+ model
c>Camera module for Raspberry pi
d>MPU6050 6dof gyroscope and accelerometer measuring module
e>DC geared motor 240rpm with encoder
f>2200Mah 11.6 v lippo battery
g>1/8 rugbby wheel 
h>lots of code.
I will be using simulink to test the control system of the bot ,  I have decided to use Cascaded PID feedback control to stabelise the system.
# CURRENT STATUS:
There is a problem with the encoders of the motrors, because of the quantization error I am unable to process the data given by encoders in PID loop which makes the loop unefective and unable to track the speed.
