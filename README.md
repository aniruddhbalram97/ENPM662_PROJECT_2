## ENPM662-PROJECT2

## DELIVERY ROBOT

## OVERVIEW
The aim of this project is to build an omni directional mobile manipulator that can be used for delivery of groceries and food.

## DEPENDENCIES
- OS : Ubuntu 20.04
- ROS Distro : ROS Noetic
- SolidWorks
- Moveit
- Gazebo

## NOTE
- In order to successfully run the simulation, we have used two plugins namely, gazebo-pkgs and general-message-pkgs. These files have also been added to the repository.

## INSTRUCTIONS TO RUN THE ROS PACKAGE 

### BUILD THE PACKAGE

Clone the repository to your workspace and build the packages:
```
cd <your_ROS_ws>/src
git clone https://github.com/Irdab2000/ENPM662_PROJECT2.git
cd .. 
catkin_make

# source your installation
source .devel/setup.bash
```
### RUN THE SIMULATION
```
roslaunch vehicle_assembly_moveit full_sim.launch
```
In another terminal :
```
cd <your ROS_ws>/src/vehicle_assembly/scripts
python3 moveit.py
```

### BEFORE RUNNING ANY VALIDATION:
```
cd <your ROS_ws>/src/vehcile_assembly/urdf
```
- Open vehicle_assembly.urdf and comment from line  2734 to 2821 and uncomment lines 2823 to 2910

```
cd <your ROS_ws>/src/vehicle_assembly/launch
```

- Open vehicle_assembly.launch and uncomment from line 39 to 50 and comment lines 30 to 40

### RUN FORWARD KINEMATICS VALIDATION:
```
roslaunch vehicle assembly vehicle_assembly.launch
```
In another terminal :
```
cd <your ROS_WS>/src/vehicle_assembly/scripts
python3 fk_validation.py
```

### RUN INVERSE KINEMATICS VALIDATION:
```
roslaunch vehicle assembly vehicle_assembly.launch
```
In another terminal :
```
cd <your ROS_WS>/src/vehicle_assembly/scripts
python3 inK_validation.py
```
### RUN WORKSPACE STUDY VALIDATION:
```
roslaunch vehicle assembly vehicle_assembly.launch
```
In another terminal :
```
cd <your ROS_WS>/src/vehicle_assembly/scripts
python3 workspace.py
```
### SIMULATION VIDEOS 
- https://drive.google.com/file/d/1qItt6UyC16aCXbElswLgnpFnVTCxZOQI/view?usp=sharing

