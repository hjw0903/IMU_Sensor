## Setting IMU sensor on the Raspberry Pi.

Created on Wed Jul 23 13:19 2018  
@author: Jungwon Hwang  

  
Installation of dependencies  
sudo apt-get install i2c-tools   
sudo apt-get install cmake  
sudo apt-get install python-dev  
sudo apt-get install octave // for calibration  
  
This following command is needed to fix the error message: Error: missing `server' JVM at `/usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/server/libjvm.so'.  
  
cp -r /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/client /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/server   
  
git clone https://github.com/richardstechnotes/RTIMULib2.git  
  
cd RTIMULib2/Linux/RTIMULibCal  
make -j4  
sudo make install  
  
#copy the compiled out put to our project  
#  
cp -r RTIMULib2/RTEllipsoidFit ../  
  
> sudo nano /etc/modules  
: make sure that both these lines are uncommented in this file:  
    i2c‐dev  
    i2c‐bcm2708 # add this line, it is needed  
  
>sudo nano /etc/udev/rules.d/90‐i2c.rules  
: add  
KERNEL=="i2c‐[0‐7]",MODE="0666"  
  
>sudo nano /boot/config.txt  
: add at the botto  
dtparam=i2c1_baudrate=400000dt  
  
# uncomment the following two lines  
dtparam=i2c_arm=on  
dtparam=spi=on  
  

>sudo reboot  
  
pi@raspberrypi:~/wheelchair_project/wheelchair_files $ ls /dev/i*  
/dev/i2c-1  /dev/initctl  
  
/dev/input:  
mice  
pi@raspberrypi:~/wheelchair_project/wheelchair_files $ cd RTEllipsoidFit/  
pi@raspberrypi:~/wheelchair_project/wheelchair_files/RTEllipsoidFit $ i2cdetect -y 1  
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f  
00:          -- -- -- -- -- -- -- -- -- -- -- -- --   
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e --   
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --   
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --   
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --   
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --   
60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --   
70: -- -- -- -- -- -- -- --                           
pi@raspberrypi:~/wheelchair_project/wheelchair_files/RTEllipsoidFit $ i2cdump -y 0x6b  
Error: No address specified!  
Usage: i2cdump [-f] [-y] [-r first-last] I2CBUS ADDRESS [MODE [BANK [BANKREG]]]  
  I2CBUS is an integer or an I2C bus name  
  ADDRESS is an integer (0x03 - 0x77)  
  MODE is one of:  
    b (byte, default)  
    w (word)  
    W (word on even register addresses)  
    s (SMBus block)  
    i (I2C block)  
    c (consecutive byte)  
    Append p for SMBus PEC  
pi@raspberrypi:~/wheelchair_project/wheelchair_files/RTEllipsoidFit $ i2cdump -y 1 0x6b  
No size specified (using byte-data access)  
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef  
00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 68    ...............h  
10: 00 00 00 00 00 60 ff 00 5c 01 64 04 d9 ff 38 38    .....`..\?d??.88  
20: 00 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00    ..?.............  
30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................  
40: 64 51 a0 21 30 af e6 40 80 c0 35 0a 9b 7a 93 2c    dQ?!0??@??5??z?,  
50: 08 12 48 bf 80 bf 87 c8 92 0d 46 61 f2 06 26 2b    ??H???????Fa??&+  
60: e4 70 c1 03 07 00 0a 2e 2a 21 80 c3 a7 65 05 20    ?p???.?.*!???e?   
70: 59 61 00 00 80 58 66 07 01 01 00 00 00 40 c0 00    Ya..?Xf???...@?.  
80: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................  
90: 00 00 00 00 00 00 f6 00 5c 01 64 04 d9 ff 00 00    ......?.\?d??...  
a0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................  
b0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................  
c0: 00 00 00 00 02 3c 24 4c 1c 22 00 00 00 00 00 00    ....?<$L?"......  
d0: 00 00 00 00 ff 01 00 00 00 00 00 00 00 00 00 00    .....?..........  
e0: 07 00 00 00 07 00 00 00 00 10 00 00 00 00 00 00    ?...?....?......  
f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0    ...............?   

  
git clone https://github.com/OldCC/scripts.git  
  
> cd /home/pi/wheel/scripts/  
> sudo python imu.py  
// This code uses the sensor fusion data. It applies the filters to get an accurate data like as Kalman‐  
filtered quaternion.  
# If you want to check the data, you can add the code before time.sleep().  
print("Heading: %f" %(heading)) # Add for checking the value  
print("ypr: %f %f %f" %(yaw, pitch, roll))  
  
'''  
