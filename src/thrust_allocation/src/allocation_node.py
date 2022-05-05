#!/usr/bin/env python
import rospy
from allocation import loop
from lib import thrusterNodeInit, nodeEnd
import os 
import yaml

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/thrust_allocation/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)


if __name__ == '__main__':

    thrusterNodeInit()
    r = rospy.Rate(params["runfrequency"]) # Usually set to 100 Hz
    
    while not rospy.is_shutdown():
        
        loop()
        r.sleep()
    
    nodeEnd()