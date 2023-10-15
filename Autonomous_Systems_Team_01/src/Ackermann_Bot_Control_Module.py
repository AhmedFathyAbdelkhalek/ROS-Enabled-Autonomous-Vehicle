#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
# from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32, String
import time
from scipy.interpolate import interp1d


#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Control_Module') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('control_signals', String, queue_size=1)
rate = rospy.Rate(35) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Suscribers Variables
Steer_Cont = 90
Vel_Cont = 0
Prev_Vel = 0

servo_angle = interp1d([60,120],[40,132])

speed_interp = interp1d([0,3.8],[0,1])
#######################################################################

#######################################################################
#ROS Subscriber Code for Steering
def callback_control_steering(data):
    global Steer_Cont	#Identify msg variable created as global variable
    global sub1		#Identify a subscriber as global variable
    Steer_Cont = data.data
    Steer_Cont = 90 - Steer_Cont 
    Steer_Cont = servo_angle(Steer_Cont)
    if Steer_Cont >= 125:
        Steer_Cont = 120
    if Steer_Cont <= 45:
        Steer_Cont = 45

sub1 = rospy.Subscriber('/Control_Action_Steering', Float32, callback_control_steering)
#######################################################################

#######################################################################
#ROS Subscriber Code for Velocity
def callback_control_velocity(data):
    global Vel_Cont	#Identify msg variable created as global variable
    global sub2	#Identify a subscriber as global variable
    Vel_Cont = data.data
    if Vel_Cont <= 0:
        Vel_Cont = 0
    elif Vel_Cont >= 1:
        Vel_Cont = 1

sub2 = rospy.Subscriber('/Control_Action_Driving_Velocity', Float32, callback_control_velocity)
#######################################################################
#########################################################################################################

#########################################################################################################
#Simulation While Loop
st_time = time.time()
vel_and_heading_msg = 0
#######################################################################
while 1 and not rospy.is_shutdown():
    end_time = time.time()
    tau  = end_time-st_time
    st_time = time.time() 

    vel_and_heading_msg = "{},{}".format(Vel_Cont, Steer_Cont)

    print(vel_and_heading_msg)

    pub1.publish(vel_and_heading_msg.encode())	#Publish msg
    Prev_Vel = Vel_Cont
    rate.sleep()

#######################################################################
#########################################################################################################

