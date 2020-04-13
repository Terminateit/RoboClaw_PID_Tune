#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from roboclaw import Roboclaw
import threading
from math import sin, cos
import time
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import PoseStamped, TwistStamped

from dynamic_reconfigure.msg import Config # dynamics reconfigure

from threading import Thread, Lock
import sys 


pi = 3.1415
tick_rev = 2256 #2421

port = sys.argv[1]
rc = Roboclaw(port, 115200)
rc.Open()
address = 0x80

mutex = Lock()

config = Config() # dynamics reconfigure

class PIDTuning_RoboClaw(Thread):
    def __init__(self):
        Thread.__init__(self)

    def pid_cb(self, data): # dynamics reconfigure callback
        global config
        config = data

    def Manual(self):
        rate = rospy.Rate(50)

        pub_sb = rospy.Subscriber('/pid_tuning/parameter_updates', Config, self.pid_cb)

        while not rospy.is_shutdown():
            if len(config.doubles) > 0:

                for i in range(len(config.doubles)):
                    if config.doubles[i].name == 'k_p':
                        self.k_p = config.doubles[i].value
                    if config.doubles[i].name == 'k_d':
                        self.k_d = config.doubles[i].value
                    if config.doubles[i].name == 'k_i':
                        self.k_i = config.doubles[i].value
            else:
                self.k_p = self.k_d = self.k_i = 5            

            print('k_p=',self.k_p, ', k_d =',self.k_d, ', k_i =',self.k_i)  
            mutex.acquire()

            #----# Uncomment lines below in order to work with RoboClaw #----#

            # rc.SetM1VelocityPID(address, self.k_p, self.k_d, self.k_i, tick_rev)
            # rc.SetM2VelocityPID(address, self.k_p, self.k_i, self.k_d, tick_rev)

            #----#
            
            # print('M1-PID-Velocity: ',rc.ReadM1VelocityPID(address), '\n')
            # print('M2-PID-Velocity: ',rc.ReadM1VelocityPID(address), '\n')

            mutex.release()
            rate.sleep()   

if __name__ == '__main__':
    print("start PID Tuning")
    PID_Tuning = PIDTuning_RoboClaw()

    rospy.init_node('PID_Tuning', anonymous=True)
    
    PID_Tuning.Manual()
    while True:
        try:
            time.sleep(0.01)
        except KeyboardInterrput:
            PID_Tuning.join()
            break
