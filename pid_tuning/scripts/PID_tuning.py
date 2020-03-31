#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from roboclaw import Roboclaw
from threading import Thread, Lock
from math import sin, cos
import time
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import PoseStamped, TwistStamped

from dynamic_reconfigure.msg import Config # dynamics reconfigure



pi = 3.1415
QPPS_rev = 2256

rc = Roboclaw("/dev/ttyACM0", 115200)
rc.Open()
address = 0x80

# mutex = Lock()
# L = rospy.get_param("base/width")
# R = rospy.get_param("wheel_big/radius")

config = Config() # dynamics reconfigure

def pid_cb(data): # dynamics reconfigure callback
    global config
    config = data

pid_sub = rospy.Subscriber('/pid_tuning/parameter_updates', Config, pid_cb) # dynamics reconfigure subscriber

rospy.init_node('pid_coefficients_tuning', anonymous=True)

rate = rospy.Rate(100.0)

# wait for connection
k_p = 0
k_d = 0
k_i = 0
while not rospy.is_shutdown():
    for i in range(len(config.doubles)):
        if config.doubles[i].name == 'k_p':
            k_p = config.doubles[i].value
        if config.doubles[i].name == 'k_d':
            k_d = config.doubles[i].value
        if config.doubles[i].name == 'k_i':
            k_i = config.doubles[i].value

    # print('k_p =',k_p, ', k_d =',k_d, ', k_i =',k_i)  
    rc.SetM1PositionPID(address, k_p, k_i, k_d, 1000, 0.5, 0, 1000)
    rc.SetM2PositionPID(address, k_p, k_i, k_d, 1000, 0.5, 0, 1000)

    print('M1-PID: ',rc.ReadM1PositionPID(address), '\n')
    print('M2-PID: ',rc.ReadM2PositionPID(address), '\n')

    rc.SetM1VelocityPID(address, k_p, k_i, k_d)
    
    rate.sleep()      

