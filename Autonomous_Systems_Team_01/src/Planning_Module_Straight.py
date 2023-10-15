#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float32, Float32MultiArray
import time

#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Planning_Module') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Longitudinal_Driving_Velocity', Float32, queue_size=1)
pub2 = rospy.Publisher('/Lateral_Distance', Float32, queue_size=1)
rate = rospy.Rate(35) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
actual_position_and_vel = Float32MultiArray()
actual_position_and_vel.data = [0,0,0,0]
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity

def callback_pos(data):
  global actual_position_and_vel
  global sub_pos
  actual_position_and_vel.data = data.data

sub_pos = rospy.Subscriber('actual_position_and_vel', Float32MultiArray, callback_pos)
#######################################################################
#########################################################################################################

#########################################################################################################
V_Des = 0.4
Lane_Des = 0
#######################################################################
#Simulation While Loop
st_time = time.time()
#######################################################################
while 1 and not rospy.is_shutdown():
  st_time = time.time()
 
  V_Act = actual_position_and_vel.data[3]

  print(actual_position_and_vel.data)
  if (actual_position_and_vel.data[0] < 1.0):
    V_Des = 0.4
    Lane_Des = 0
  elif (actual_position_and_vel.data[0] >= 10):
    V_Des = 0

  print("Desired Veloctiy: %s, Desired Lane: %s"%(V_Des,Lane_Des))
  pub1.publish(V_Des)	#Publish msg
  pub2.publish(Lane_Des)	#Publish msg
  rate.sleep()
  end_time = time.time()
  tau  = end_time-st_time
#######################################################################
#########################################################################################################