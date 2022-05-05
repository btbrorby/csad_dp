#!/usr/bin/env python

import rospy 

from dynamic_reconfigure.server import Server
from gain_server.cfg import gainsConfig


def callback(config, level):
    """
    The callback function prints the updated gains of the observer and controller everytime the 
    """
    rospy.loginfo("""Reconfigure Request: \n 
    Observer gains: \n 
    --------------- \n
    L1: [{L1}] \n 
    L2: [{L2}] \n
    L3: [{L3}] \n
    L4: [{L4}] \n
    --------------- \n
    Controller gains: \n
    --------------- \n
    Kp: [{Kp}]\n
    Kd: [{Kd}]\n 
    Ki: [{Ki}]\n
    Kb: [{Kb}]\n
    mu: {mu}\n 
    U_ref: {U_ref} \n""".format(**config))
    return config

if __name__=="__main__":
    node = rospy.init_node("gain_server", anonymous = False)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        
        srv = Server(gainsConfig, callback)
        r.sleep()
        rospy.spin()
    
    node.destroy_node()