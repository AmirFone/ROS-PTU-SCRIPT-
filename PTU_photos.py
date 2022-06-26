#!/usr/bin/env python
# 
#
# Program that
# moves the pan tilt unit
# to close of program by ^C
# dml 2022
#
#




#script wrtie my Amir H for Fordham university's Computer Vision and Robotics Lab.
#Email:amirmd6000@gmail.com
import time
import math
import random
import rospy # needed for ROS
import numpy as np # for map arrays
import matplotlib.pyplot as plt
#from matplotlib.image import imread  


# ROS message types
from sensor_msgs.msg import JointState       # ROS DP PTU command
from geometry_msgs.msg import Twist      # ROS Twist message



import cv2
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

gCurrentImage = CompressedImage() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object
gLoc = [0,0,0]          # location of robot


# ROS topics

motionTopic='/P2/RosAria/cmd_vel' 
poseTopic = '/P2/RosAria/pose'
imageTopic = '/P2/camera/color/image_raw/compressed' 


motionTopic='/P2/RosAria/cmd_vel' 
ptuCmdTopic = '/P2/ptu/cmd'          # check that these are right
ptuPosTopic = '/P2_joint_states'

#globals

gPTU = [0.0, 0.0]
gCtr=0
gWait=False
gImageList =[]

# get current PT position from PTU
# ignores velocity, effort and finished flag
#
def ptuCallback(data):
    global gPTU,gWait,gCtr
    gPTU=data.position
    gCtr += 1
    if gCtr>2100000 and not gWait: # output the PTU position every few secs
        print("PT position:",gPTU)
        gCtr=0
    return


def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge
    #gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    np_arr=np.fromstring(img.data,np.uint8)
    gCurrentImage=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
    return
#
# publish a command to move the ptu unit
# pub=publisher, seq = message sequence number (monotinic increasing)
#
def camera_view():
    count=0
    for i in gImageList:
    	cv2.imshow('Camera',i)
    	cv2.waitKey(1)
        temp="p2_imagePhotoa"+str(count)+".jpg"
    	cv2.imwrite(temp,i)
        count+=1
	rospy.sleep(5)
    return
def sendPTcommand(pub,p,t,seq):
    msg = JointState()
    msg.header.frame_id='0'
    msg.header.stamp=rospy.Time.now()
    msg.name=["pan","tilt"]
    msg.position = [p,t]
    msg.velocity=[2,2]
    msg.effort=[1,1]
    pub.publish(msg)

    return seq+1

#
# move the PT unit
# ask user for input pan and tilt settings and
# send them to the PT

def PTU_test_node():
    global gCurrentImage,Image
    #list of angles
    A=[0,120,60,0,-60,-120,-180] 
    '''send pt commands to the pt unit'''
    global gWait
    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('PTU_test_Node', anonymous=True)

    # register as a ROS publisher for the PTU position
    ptu_pub = rospy.Publisher(ptuCmdTopic, JointState, queue_size=10)
    ptu_sub = rospy.Subscriber(ptuPosTopic, JointState,ptuCallback)
    image_sub = rospy.Subscriber(imageTopic,CompressedImage,callbackImage)

    msg_seq_num=0 
    i=0
    while not rospy.is_shutdown():
        if i==7:
            break
	if i>0:
		print("Paning to: ",A[i])

        pan= math.radians(A[i])
        #going to the corresponding angle from the "A" list 
        msg_seq_num = sendPTcommand(ptu_pub,pan,0,msg_seq_num)
        #waiting for PTU to move 
        rospy.sleep(3)
	#cv2.imshow('Camera',gCurrentImage)
    	#cv2.waitKey(10)
        #saving the current image to the image list
        gImageList.append(gCurrentImage)
        i+=1

    msg_seq_num = sendPTcommand(ptu_pub,0,0,msg_seq_num)
    #displaying the image     
    camera_view()
    return

#
# This function is called by ROS when you stop ROS
# Here we use it to send a zero velocity to robot
# in case it was moving when you stopped ROS
#

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    return



#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
	PTU_test_node()
        rospy.on_shutdown(callback_shutdown)
    except rospy.ROSInterruptException:
        pass

