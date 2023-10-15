#! /usr/bin/env python 

import rospy  
from std_msgs.msg import Float32, Float32MultiArray
import time
import math

x_current, y_current = 0, 0
yaw = 0.0 
linear_vel = 0.0 
actual_pos = Float32MultiArray()
actual_pos.data = [0,0,0,0]

def get_yaw(data):
    global yaw 
    yaw = data.data 

def get_v(data):
    global linear_vel 
    linear_vel = data.data 

rospy.init_node("Localization")

rate = rospy.Rate(35)
rospy.Subscriber("actual_heading" , Float32 , get_yaw)
rospy.Subscriber("actual_velocity" , Float32 , get_v)

pub_actual_pos = rospy.Publisher("actual_position_and_vel", Float32MultiArray , queue_size =1)

dt_timer = 0

while not rospy.is_shutdown(): 
    dt = time.time() - dt_timer
    dt_timer = time.time()
    x_current += dt*(linear_vel * math.cos(math.radians(yaw)))
    y_current += dt*(linear_vel * math.sin(math.radians(yaw)))

    actual_pos.data = [x_current,y_current,yaw,linear_vel]
    pub_actual_pos.publish(actual_pos)

    print("Velocity: %s m/s, X: %s, Y: %s, Theta: %s"
         %(round(linear_vel,3),round(x_current,3),round(y_current,3),round(yaw,3))) 
    rate.sleep()