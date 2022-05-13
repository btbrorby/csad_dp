#!/usr/bin/env python
import rospy
from Observer import loop
from lib import observerNodeInit, nodeEnd
import os 
import yaml
import matplotlib.pyplot as plt
import time as tic

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/observer/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)



if __name__ == '__main__':
    observerNodeInit()
    r = rospy.Rate(params["runfrequency"]) # Usually set to 100 Hz
    t0 = tic.time()
    while not rospy.is_shutdown():
        
        loop()
        r.sleep()
        print("OBSERVER time", tic.time()-t0)
    
    nodeEnd()