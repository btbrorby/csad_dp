#!/usr/bin/env python
import rospy
from accFeedforwardController import loop
from lib import controllNodeInit, nodeEnd
import yaml
import os
import time as tic

t0 = tic.time()

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)


if __name__ == '__main__':

    controllNodeInit()
    r = rospy.Rate(params["runfrequency"])
    
    print("CONTROLLER time", tic.time() - t0)
    while not rospy.is_shutdown():
        loop()
        r.sleep()
        
    
    nodeEnd()