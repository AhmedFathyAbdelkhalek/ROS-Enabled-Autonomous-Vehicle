# ROS-Enabled Autonomous Vehicle
This is my code to develop a ROS-enabled autonomous vehicle. The code runs on a Raspberry Pi 4 with ROS Melodic. 

The car utilizes the following hardware electronics shown in the schematic:

![diagram](https://github.com/AhmedFathyAbdelkhalek/ROS-Enabled-Autonomous-Vehicle/assets/89396236/91327dcb-4194-4185-b973-71ec617fdd9a)

The Arduino uses standard PyFirmata to receive controller actions from the Pi and send them to the motors. There's a DC motor for longitudinal control and a micro servo for lateral control.