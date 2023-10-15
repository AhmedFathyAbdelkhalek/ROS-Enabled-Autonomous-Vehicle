#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
# from geometry_msgs.msg import Pose,Twist
# from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import time

#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Control_Module_Longitudinal_Motion') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub_velocity = rospy.Publisher('/Control_Action_Driving_Velocity', Float32, queue_size=1)
rate = rospy.Rate(35) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
actual_position_and_vel = Float32MultiArray() # Pos X, Pos Y, Yaw
actual_position_and_vel.data = [0,0,0,0]
v_desired = 0
vel_error_sum = 0
derivative = 0
delta = 0
error_prev = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity

def callback_pos(data):
  global actual_position_and_vel
  global sub_pos
  actual_position_and_vel.data = data.data

# sub_vel = rospy.Subscriber('actual_velocity', Float32, callback_vel)
sub_pos = rospy.Subscriber('actual_position_and_vel', Float32MultiArray, callback_pos)
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_des_vel(data):
  global v_desired	#Identify msg variable created as global variable
  global sub_des_vel#Identify a subscriber as global variable
  v_desired = data.data
  # print('v_desired: '+str(v_desired)+ "m/s")

sub_des_vel = rospy.Subscriber('/Longitudinal_Driving_Velocity', Float32, callback_des_vel)
#######################################################################
#########################################################################################################

#########################################################################################################
def speed_control(v_desired,v_act,tau):
  global vel_error_sum
  global error_prev
  K_p = 5
  k_i = 0.5
  k_d = 0.8
  vel_error = v_desired-v_act 
  print("ERROR: %s"%vel_error)
  vel_error_sum += (vel_error*tau)
  delta = vel_error - error_prev
  derivative = delta/tau
  u = K_p*vel_error + vel_error_sum*k_i + derivative*k_d
  print("Control Signal: %s"%u)
  # if(np.abs(u) > 2):
  #   u = 2*np.sign(u)
  v_control = v_act + tau*u
  error_prev = vel_error
  if v_desired == 0:
    v_control = 0
  return v_control
#########################################################################################################

#########################################################################################################
Wheel_Base = 0.15 #rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 
#######################################################################
#Simulation While Loop
st_time = time.time()
tau = 0.01
v_control = 0
#######################################################################
while 1 and not rospy.is_shutdown():
  st_time = time.time()

  v_act = actual_position_and_vel.data[3]
  # print('Actual Velocity: '+str(v_act))

  # print('Actual Position X: '+str(actual_position_and_vel.data[0]) + ' m ,Actual Position Y: '+str(actual_position_and_vel.data[1]) 
        # + ' m ,Actual Heading: '+ str(actual_position_and_vel.data[2]) + ' degrees')

  v_control = speed_control(v_desired,v_act,tau)
  print('Control Velocity: '+str(v_control))

  pub_velocity.publish(v_control)	#Publish msg
  rate.sleep()
  end_time = time.time()
  tau  = end_time-st_time
  #print(tau)
#######################################################################
#########################################################################################################