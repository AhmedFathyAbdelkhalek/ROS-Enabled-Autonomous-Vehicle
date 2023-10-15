#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import time

#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Control_Module_Lateral_Motion') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Steering', Float32, queue_size=1)
rate = rospy.Rate(35) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
actual_position_and_vel = Float32MultiArray() # Pos X, Pos Y, Yaw
actual_position_and_vel.data = [0,0,0,0]
desired_lane = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity

def callback_pos(data):
  global actual_position_and_vel
  global sub_pos
  actual_position_and_vel.data = data.data

sub_pos = rospy.Subscriber('actual_position_and_vel', Float32MultiArray, callback_pos)
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_des_lane(data):
  global desired_lane	#Identify msg variable created as global variable
  global sub2	#Identify a subscriber as global variable
  desired_lane = data.data
  # print('desired_lane = '+str(desired_lane))
  
sub2 = rospy.Subscriber('/Lateral_Distance', Float32, callback_des_lane)
#######################################################################
#########################################################################################################

#########################################################################################################
def stanley_control(desired_lane,pos_act,v_act):
  # K_p = 0.5
  if v_act == 0:
    v_act = 0.3
  v_act = 0.3
  global wheel_base
  K_v = 1.5
  yaw = np.radians(pos_act[2])
  x_Lane = pos_act[0] + 1*np.sign(v_act)#K_p*v_act
  Lookahead_pnt = [x_Lane,desired_lane]
  # print('Lookahead_pnt = '+str(Lookahead_pnt))
  # dx = Lookahead_pnt[0]-(pos_act[0]+wheel_base*np.cos(pos_act[2]))
  dy = Lookahead_pnt[1]-(pos_act[1]+wheel_base*np.sin(yaw))
  # L_d = np.sqrt((dx)**2 + (dy)**2)

  th_p = 0 - yaw
  try:
    steer_angle_control = th_p + np.arctan(((K_v*dy)/(v_act)))
  except:
    steer_angle_control = 0
  if(abs(steer_angle_control) >= np.radians(30)): # 30 is the max degree achievable by the vehicle's steering system
    steer_angle_control = np.sign(steer_angle_control)*np.radians(30)
  return np.degrees(steer_angle_control)
#########################################################################################################

#########################################################################################################
wheel_base = 0.15 #Check Gazebo Model Dimensions 
#######################################################################
#Simulation While Loop
st_time = time.time()
steer_angle_control = 0
#######################################################################
while 1 and not rospy.is_shutdown():
  st_time = time.time()

  v_act = actual_position_and_vel.data[3]
  
  #print('Vel_Act = '+str(v_act))

  steer_angle_control = stanley_control(desired_lane,actual_position_and_vel.data,v_act)

  pub1.publish(steer_angle_control)	#Publish msg
  rate.sleep()
  end_time = time.time()
  tau  = end_time-st_time

  #print(tau)
  #######################################################################
#########################################################################################################