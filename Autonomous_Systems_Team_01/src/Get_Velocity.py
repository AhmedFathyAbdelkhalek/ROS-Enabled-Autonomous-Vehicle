#!/usr/bin/env python

import pyfirmata
from time import sleep
import rospy
from std_msgs.msg import Float32, String
import math
import gpiozero
import time

serial_port = '/dev/ttyUSB0'
board = pyfirmata.Arduino(serial_port)

rospy.init_node("get_velocity")

encoder = gpiozero.DigitalInputDevice(27)

pin_servo = 3
pin_dir1 = 6
pin_dir2 = 7
pin_speed = 5

heading = 90 # between min and max car angles
speed = 0 # between 0 and 1
control_signals = "0.4,90"

pulse_count = 0.0
pulses_per_rev = 12.0
flag = False
RPM = 0.0

angular_vel = 0.0
wheel_radius = 0.01975 # 39.5 mm diameter

def getRPM():
  global pulse_count
  global RPM
  global pulses_per_rev
  global flag
  last_time = time.time()
  while time.time() - last_time <= 0.025:
    if encoder.value == 1:
      flag = True
    if encoder.value == 0 and flag == True:
      pulse_count +=1
      flag = False
  flag = False
  RPM = (pulse_count/pulses_per_rev)*2400
  pulse_count = 0
  # RPM = 0
  return RPM

pub = rospy.Publisher("actual_velocity",Float32 , queue_size = 1)
rate = rospy.Rate(35)

board.digital[pin_servo].mode = pyfirmata.SERVO
board.digital[pin_speed].mode = pyfirmata.PWM

def callback_control_signal(data):
  global control_signals
  global sub_cont,flag_control
  control_signals = data.data

sub_cont = rospy.Subscriber('control_signals', String, callback_control_signal)

board.digital[pin_dir1].write(0)
board.digital[pin_dir2].write(1)

while 1 and not rospy.is_shutdown():
    RPM = getRPM()
    angular_vel = (RPM*2*math.pi)/60
    linear_vel = round(angular_vel * wheel_radius,3)
    
    print("{}".format(linear_vel)+ " m/s")

    pub.publish(linear_vel)

    speed,heading = control_signals.split(',')
    speed = float(speed)
    heading = float(heading)
    board.digital[pin_servo].write(heading)
    board.digital[pin_speed].write(speed)
    rate.sleep()