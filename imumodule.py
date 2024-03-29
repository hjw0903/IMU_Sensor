import sys, getopt
import threading


import RTIMU
import os.path
import time
import math
import operator
import socket
import os
from functools import reduce

class Imu:

    imu = None
    ypr = None 

    def __init__(self):
        pass

    def getData(self):

        SETTINGS_FILE = "RTIMULib"
        s = RTIMU.Settings(SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(s)

        yawoff = 0.0
        pitchoff = 0.0
        rolloff = 0.0

        # timers
        t_read = time.time()
        t_damp = time.time()
        t_fail = time.time()
        t_fail_timer = 0.0
        t_shutdown = 0

        while (not self.imu.IMUInit()):
            now = time.time()
            imu_sentence = "$IIXDR,IMU_FAILED_TO_INITIALIZE*7C"
            if (now - t_read) > 1.0:
                t_read = now
                t_shutdown += 1
                print(t_shutdown)
                if t_shutdown > 9:
                    raise RuntimeError('Could not initialize the 9 axis module.')

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        poll_interval = self.imu.IMUGetPollInterval()

        # data variables
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        heading = 0.0
        rollrate = 0.0
        pitchrate = 0.0
        yawrate = 0.0

        # magnetic deviation
        magnetic_deviation = self.getMageticDeviation()

        # dampening variables
        t_one = 0
        t_three = 0
        roll_total = 0.0
        roll_run = [0] * 10
        heading_cos_total = 0.0
        heading_sin_total = 0.0
        heading_cos_run = [0] * 30
        heading_sin_run = [0] * 30

        for i in range(1,10):
            now = time.time()

            if (now - t_damp) > 5.0:
                if (now - t_fail) > 1.0:
                    t_one = 0
                    t_three = 0
                    roll_total = 0.0
                    roll_run = [0] * 10
                    heading_cos_total = 0.0
                    heading_sin_total = 0.0
                    heading_cos_run = [0] * 30
                    heading_sin_run = [0] * 30
                    t_fail_timer += 1
                    t_fail = now
                    t_shutdown += 1

            if self.imu.IMURead():
                data = self.imu.getIMUData()
                fusionPose = data["fusionPose"]
                Gyro = data["gyro"]
                t_fail_timer = 0.0

                if (now - t_damp) > .1:
                    roll = round(math.degrees(fusionPose[0]) - rolloff, 1)
                    pitch = round(math.degrees(fusionPose[1]) - pitchoff, 1)
                    yaw = round(math.degrees(fusionPose[2])- yawoff, 1)
                    rollrate = round(math.degrees(Gyro[0]), 1)
                    pitchrate = round(math.degrees(Gyro[1]), 1)
                    yawrate = round(math.degrees(Gyro[2]), 1)
                    if yaw < 0.1:
                        yaw = yaw + 360
                    if yaw > 360:
                        yaw = yaw - 360

                    # Dampening functions
                    roll_total = roll_total - roll_run[t_one]
                    roll_run[t_one] = roll
                    roll_total = roll_total + roll_run[t_one]
                    roll = round(roll_total / 10, 1)
                    heading_cos_total = heading_cos_total - heading_cos_run[t_three]
                    heading_sin_total = heading_sin_total - heading_sin_run[t_three]
                    heading_cos_run[t_three] = math.cos(math.radians(yaw))
                    heading_sin_run[t_three] = math.sin(math.radians(yaw))
                    heading_cos_total = heading_cos_total + heading_cos_run[t_three]
                    heading_sin_total = heading_sin_total + heading_sin_run[t_three]
                    yaw = round(math.degrees(math.atan2(heading_sin_total/30,heading_cos_total/30)),1)
                    if yaw < 0.1:
                        yaw = yaw + 360.0

                    # yaw is magnetic heading, convert to true heading
                    heading = yaw - magnetic_deviation
                    if heading < 0.1:
                        heading = heading + 360
                    if heading > 360:
                        heading = heading - 360

                    t_damp = now
                    t_one += 1
                    if t_one == 10:
                        t_one = 0
                    t_three += 1
                    if t_three == 30:
                        t_three = 0     
                    t_read = now 
                    
            ypr = [heading,yaw,pitch,roll]        
            time.sleep(poll_interval*1.0/1000.0)
        return ypr 


    def getMageticDeviation(self):
        try:        
            f = open('mag', 'r')
            magnetic_deviation = float(f.readline())
        except:
            f = open('mag', 'w')
            f.writelines("0.0516\n # This number is for Winnipeg,MB\n")    
            magnetic_deviation = float('0.0516')
        finally:
            f.close()
            return magnetic_deviation

def getImu(imu):
    global heading
    while True:
        time.sleep(0.1)
        heading = imu.getData()       

if __name__ == '__main__':
    imu = Imu()    
    heading = None
    thread_imu = threading.Thread(target=getImu, args=(imu,))
    thread_imu.start()

    while True:
        time.sleep(0.1)
        if heading is not None:
            ypr = heading
            print(heading)
            #print("hypr: %.2f %.2f %.2f %.2f" %(ypr[0],ypr[1],ypr[2],ypr[3]))
        #thread_imu.join()

