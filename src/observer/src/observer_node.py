#!/usr/bin/env python3
import rospy
from observer import loop
from lib import observerNodeInit, nodeEnd
import os 
import yaml
#from ctypes.wintypes import MSG

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/observer/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)



if __name__ == '__main__':

    observerNodeInit()
    r = rospy.Rate(params["runfrequency"]) # Usually set to 100 Hz
    
    while not rospy.is_shutdown():
        
        loop()
        r.sleep()
    
    nodeEnd()