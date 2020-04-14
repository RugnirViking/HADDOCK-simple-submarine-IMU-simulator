#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16

def _get_vector3(roll, pitch,heading):
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = heading
        return vec

def start(name):
    rospy.init_node('python_simple_sub_simulator', anonymous=True) # node is called 'python_simple_sub_simulator'
    rospy.loginfo("Loading Simple Sin Simulator")
    pub = rospy.Publisher('imu_values_topic', Vector3, queue_size=10)
    depth_pub = rospy.Publisher('depth_topic', Int16, queue_size=10)



    roll = 0
    pitch = 0
    heading = 360
    time = 0
    depth = 10


    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        time+=1
        roll = 7*math.sin(time/60.0)
        pitch = 7*math.cos(time/86.0)
        if roll>0:
            heading += 0.1
        else:
            heading-=0.1

        if pitch<0:
            depth += 0.1
        else:
            depth-=0.1
            if depth<0:
                depth=0
			
        if heading>360:
            heading-=360
        if heading<-360:
            heading+=360

        rospy.loginfo("Sending Simulated IMU Data. Roll: "+str(round(roll))+" Pitch: "+str(round(pitch))+" Heading: "+str(round(heading)))
        pub.publish(_get_vector3(roll,pitch,heading))

        rospy.loginfo("Sending Simulated Depth Data. Depth: "+str(round(roll)))
        depth_msg = Int16()
        depth_msg.data = depth
        depth_pub.publish(depth_msg)
        rate.sleep()
