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

def callbackCheck(msgRH,msgLH): 

    if (msgRH.point.x < 0.17 or msgLH.point.x < 0.17):
        print("Issue on X: Table too close")
    elif (msgRH.point.x > 0.29 or msgLH.point.x > 0.29):
        print("Issue on X : Table too far")
    elif (msgRH.point.y < 0.17 or msgLH.point.y < -0.15):
        print("Issue on Y: Table too far on the right")  
    elif (msgRH.point.y > 0.25 or msgLH.point.y > -0.06):
        print("Issue on Y: Table too far on the left")               
    else :
        print("It's alright ! Let's go ! Enter to start dynamic graph ...")
        subRH.unregister()
        subLH.unregister() 

def callback(msgRH,msgLH):

    configuration_id = 0

    stringRH = ["target = (" + str(0.16+msgRH.point.x) + "," + str(-0.25+msgRH.point.y) + ",",str(0) + "," + str(-1.5+msgRH.point.z) + ")"]
    stringLH = ["target = (" + str(0.16+msgLH.point.x) + "," + str(0.25+msgLH.point.y) + ",",str(0) + "," + str(1.5+msgLH.point.z) + ")"]


    if (msgRH.point.x < 0.15):
        targetRH = stringRH[0] + str(0.74) + "," + str(1.55) + "," + stringRH[1]
    else:
        configuration_id += 1
        targetRH = stringRH[0] + str(0.75) + "," + str(1.6) + "," + stringRH[1]    

    if (msgLH.point.x < 0.15):
        targetLH = stringLH[0] + str(0.74) + "," + str(-1.55) + "," + stringLH[1]
    else:
        configuration_id += 2
        targetLH = stringLH[0] + str(0.75) + "," + str(-1.6) + "," + stringLH[1] 

    if configuration_id == 1: # Only RH is far from the table
        targetRH_tmp = "target = (" + str(0.16+msgLH.point.x) + "," + str(-0.25+msgRH.point.y) + "," \
        + str(0.74) + "," + str(1.55) + "," + str(0) + "," + str(-1.5) + ")"    
        print ("tmp " + targetRH_tmp)
        runCommandClient(targetRH_tmp)
        runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))") 
        print (targetLH)
        runCommandClient(targetLH)
        runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))") 

        time.sleep(4)      
        print ("-----------------------------------------------------------")

        print (targetRH)
        runCommandClient(targetRH)
        runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    elif configuration_id == 2: # Only LH is far from the table
        print (targetRH)
        runCommandClient(targetRH)
        runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")
        targetLH_tmp = "target = (" + str(0.16+msgRH.point.x) + "," + str(0.25+msgLH.point.y) + "," \
        + str(0.74) + "," + str(-1.6) + "," + str(0) + "," + str(1.5) + ")"          
        print ("tmp " + targetLH_tmp)
        runCommandClient(targetLH_tmp)
        runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))") 

        time.sleep(4)      
        print ("-----------------------------------------------------------")    

        print (targetLH)
        runCommandClient(targetLH)
        runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")

    else : # Both RH and LH are either near or far from the table
        if(configuration_id == 3): # Both RH and LH are far from the table
            targetRH_tmp = "target = (" + str(0.16+msgRH.point.x/2) + "," + str(-0.25+msgRH.point.y) + "," \
            + str(0.74) + "," + str(1.55) + "," + str(0) + "," + str(-1.5) + ")"    
            print ("tmp " + targetRH_tmp)
            runCommandClient(targetRH_tmp)
            runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")
            targetLH_tmp = "target = (" + str(0.16+msgLH.point.x/2) + "," + str(0.25+msgLH.point.y) + "," \
            + str(0.74) + "," + str(-1.55) + "," + str(0) + "," + str(1.5) + ")"          
            print ("tmp " + targetLH_tmp)
            runCommandClient(targetLH_tmp)
            runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))") 

            time.sleep(4)      
            print ("-----------------------------------------------------------")   

        print (targetRH)
        runCommandClient(targetRH)
        runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")         
        print (targetLH)
        runCommandClient(targetLH)
        runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")

    subRH.unregister()
    subLH.unregister() 

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

    # Check table distance
    rospy.init_node('TablePrehension', anonymous=True)

    global subRH
    subRH = message_filters.Subscriber("cmd_right_hand_sim", PointStamped) 

    global subLH
    subLH = message_filters.Subscriber("cmd_left_hand_sim", PointStamped)

    ts = message_filters.TimeSynchronizer([subRH,subLH], 1)
    ts.registerCallback(callbackCheck)
    
    # If no issue let's start
    raw_input()
    runCommandStartDynamicGraph() 

    print("...done")

    time.sleep(2)

    runCommandClient("target = (0.0,0.0,1.0,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")
    runCommandClient("sot.push(taskC.task.name)")    

    time.sleep(3)    

    runCommandClient("target = (0.15,0.4,0.85,-0.5,0.0,0.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,0.9,0.01,0.9))")
    runCommandClient("sot.push(taskLH.task.name)")    
    runCommandClient("target = (0.15,-0.4,0.85,0.5,0.0,-0.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,0.9,0.01,0.9))")
    runCommandClient("sot.push(taskRH.task.name)")

    time.sleep(3)

    runCommandClient("target = (-0.025,0.0,0.9,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")

    time.sleep(3)

    runCommandClient("target = (0.15,0.3,0.76,-1.0,0.0,1.0)")
    runCommandClient("gotoNd(taskLH,target,'111111',(1.0,0.9,0.01,0.9))")    
    runCommandClient("target = (0.15,-0.3,0.76,1.0,0.0,-1.0)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(1.0,0.9,0.01,0.9))")

    time.sleep(5)

    runCommandClient("target = (0.16,0.25,0.74,-1.55,0.0,1.5)")
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.16,-0.25,0.74,1.55,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    time.sleep(10)

    subRH = message_filters.Subscriber("cmd_right_hand_sim", PointStamped) 
    subLH = message_filters.Subscriber("cmd_left_hand_sim", PointStamped)

    ts = message_filters.TimeSynchronizer([subRH,subLH], 1)
    ts.registerCallback(callback)

    time.sleep(7)

    runCommandClient("q[35]=-0.3")
    runCommandClient("robot.taskUpperBody.feature.posture.value = q")    
    runCommandClient("q[27]=-0.3")   
    runCommandClient("robot.taskUpperBody.feature.posture.value = q")

    time.sleep(5)

    runCommandClient("target = (0.25,0.25,0.8,-1.5,0.0,1.5)") 
    runCommandClient("gotoNd(taskLH,target,'111111',(3.0,2.9,0.01,0.9))")    
    runCommandClient("target = (0.25,-0.25,0.8,1.5,0.0,-1.5)")    
    runCommandClient("gotoNd(taskRH,target,'111111',(3.0,2.9,0.01,0.9))")

    rospy.wait_for_service('/gazebo/apply_body_wrench')

    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = 47
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    
    apply_body_wrench(body_name = "my_table::link_0",reference_frame = "my_table::link_0",\
    wrench = wrench,duration = rospy.Duration(-1))

    time.sleep(3)

    runCommandClient("target = (0.0,0.0,0.9,0.0,0.0,0.0)")
    runCommandClient("gotoNd(taskC,target,'111111',(1.0,0.9,0.02,0.9))")
    runCommandClient("sot.push(taskC.task.name)")

    wrench.force.z = 49.25  
    apply_body_wrench(body_name = "my_table::link_0",reference_frame = "my_table::link_0",\
    wrench = wrench,duration = rospy.Duration(-1))

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

