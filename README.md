# HospitalHunter

# Using 
• Python <br>
• CoppeliaSim and the simulated MyT <br>
• ROS2 <br>
• Linux <br>

# Project Description
The goal of this project was to identify identify objects through their colours. The robot would explore the rooms and try to detect the colour of objects. A mini Database with the colours and the objects name was made. By identifying the colours, the robot would be able to know what the item is.


# To run 
## Terminal #1:
### (EVERYTIME CHANGES ARE MADE TO THE PACKAGE CD TO THE ~/dev_ws/ folder): <br>
•	cd ~/dev_ws <br>
• colcon build <br>
•	source ~/dev_ws/install/setup.bash <br>

## Terminal #2:
### to get the coppelia sim running
cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04<br>
bash coppeliaSim.sh <br>
load the scene that can be found in Scene/final.ttm <br>
press the start button to start the simulation <br>


## Terminal #3:
source ~/dev_ws/install/setup.bash <br>
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0


## Terminal #4:
### To run the compulsary.launch.xml (for task 3):
if you want to run the other task, copy the line below, but just change the name of the launch file as seen in the launch file folder e.g compulsary.launch.xml -> controller_node2.launch.xml  <br>  <br>
• source ~/dev_ws/install/setup.bash <br>
• ros2 launch HospitalHunter compulsary.launch.xml thymio_name:=thymio0 <br>


## Terminal #5:
#### to see the visualisation and camera from the robot pov
ros2 topic list (to see the topics of the robot - thymio0) <br>
rqt


#  Future Improvements:
•	Computer Vision aspect with the connection of both the opencv library and the robot camera.<br>
•	Finding a better robot to use because the thymio was too small for detecting certain things in the scene.<br>
•	Integrating object detect : I went with colour detection for identifying the objects. <br>
