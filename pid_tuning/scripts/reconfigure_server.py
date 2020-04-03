#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from pid_tuning.cfg import ReconfigureConfig

def callback(config, level):
    rospy.loginfo("""Configure : {k_p}, {k_d}, {k_i}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("gains", anonymous = True)
    srv = Server(ReconfigureConfig, callback)
    rospy.spin()
