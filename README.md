# ROS-Enabled Autonomous Vehicle
This is my code to develop a ROS-enabled autonomous vehicle. The code runs on a Raspberry Pi 4 with ROS Melodic. 

The car utilizes the following hardware electronics shown in the schematic:

![diagram](https://github.com/AhmedFathyAbdelkhalek/ROS-Enabled-Autonomous-Vehicle/assets/89396236/5e08145b-5d8d-4990-b7e7-ed099c4345bf)

The Arduino uses standard PyFirmata to receive controller actions from the Pi and send them to the motors. There's a DC motor for longitudinal control and a micro servo for lateral control.

The car has two modes of operation:
- Driving straight for 10 meters
- Lane swap at 0, 4, and 8 meters and stopping at 10 meters