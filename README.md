# moveit_hardware_integration
This repository contains the test done on integrating DJI RS3 pro gimal(hardware) with moveit(software). 

- At first digital twin of the dji rs3 pro gimbal was created and got transferred into the ROS domain as URDF and then proper controllers were assinged.
- I used ros_control_boilerplate package to connect the joints from the simulated environments to connect the real joints of the hardware
- I had the teo options to control the hardware, one is through moveit software and then the other is through moveit API
- I wanted to implement a tracking system in the ROS environment using gimbal and camera
- So, I used perception model and identify human. when human was identified the model will return the angles of each joint of the gimbal so that the gimbal will face the camera towards him
- The I subscribed these topic from the moveit API python file and integrated the whole system
