#!/usr/bin/python
import sys
import rospy
from math import *

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

from geometry_msgs.msg import PointStamped, Wrench
import message_filters
from gazebo_msgs.srv import ApplyBodyWrench

import time

def launchScript(code,title,description = ""):
    raw_input(title+':   '+description)
    print(title)
    print(code)
    for line in code:
        if line != '' and line[0] != '#':
            print line
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print answer
    print("...done with "+title)

# Waiting for services
try:
    print("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    print("...ok")

    print("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    print("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    initCode = open( "SotInitSim.py", "r").read().split("\n")
    
    print("Stack of Tasks launched")

    launchScript(initCode,'initialize SoT')
    
    # If no issue let's start
    print("Ready to go ?")
    raw_input()
    runCommandStartDynamicGraph() 

    print("...done")

    time.sleep(2)

    runCommandClient("target = (0.0,0.0,1.0,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")
    runCommandClient("sot.push(taskC.task.name)")    

    print("Move hands")
    raw_input() 

    runCommandClient("target = (0.15,0.4,0.85,-0.5,0.0,0.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,0.9,0.01,0.9))")
    runCommandClient("sot.push(taskLH.task.name)")    
    runCommandClient("target = (0.15,-0.4,0.85,0.5,0.0,-0.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,0.9,0.01,0.9))")
    runCommandClient("sot.push(taskRH.task.name)")

    print("Go down")
    raw_input()

    runCommandClient("target = (-0.025,0.0,0.9,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")

    print("Move hands")
    raw_input() 

    runCommandClient("target = (0.15,0.3,0.76,-1.0,0.0,1.0)")
    runCommandClient("gotoNd(taskLH,target,'111111',(1.0,0.9,0.01,0.9))")    
    runCommandClient("target = (0.15,-0.3,0.76,1.0,0.0,-1.0)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(1.0,0.9,0.01,0.9))")

    time.sleep(5)

    runCommandClient("target = (0.16,0.25,0.74,-1.55,0.0,1.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.16,-0.25,0.74,1.55,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    time.sleep(5)

    runCommandClient("target = (0.20,0.25,0.74,-1.55,0.0,1.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.20,-0.25,0.74,1.55,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")    

    print("Close hands")
    raw_input() 

    runCommandClient("q[35]=-0.3")
    runCommandClient("robot.taskUpperBody.feature.posture.value = q")    
    runCommandClient("q[27]=-0.3")   
    runCommandClient("robot.taskUpperBody.feature.posture.value = q")

    time.sleep(3)
    print("Ready to lift table with Pyrene ?")
    raw_input()

    runCommandClient("target = (0.25,0.25,0.8,-1.5,0.0,1.5)") 
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.25,-0.25,0.8,1.5,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    time.sleep(3)

    runCommandClient("target = (0.0,0.0,0.9,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")
    runCommandClient("sot.push(taskC.task.name)")

    runCommandClient("target = (0.2,0.25,0.9,-1.5,0.0,1.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.2,-0.25,0.9,1.5,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    time.sleep(3)

    runCommandClient("target = (0.0,0.0,1.1,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")
    runCommandClient("sot.push(taskC.task.name)")

    runCommandClient("target = (0.2,0.25,1.0,-1.5,0.0,1.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.2,-0.25,1.0,1.5,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)

