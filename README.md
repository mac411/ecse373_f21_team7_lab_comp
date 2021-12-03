# ecse373_f21_team7_lab_comp
This README will describe how to use the `lab_comp` ROS package.

## Prerequisites
- Install Ubuntu 18.04 and ROS Melodic (or appropriate equivalents).
- Install ARIAC 2019:

        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list'
        
        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        
        sudo apt-get update
        
        sudo apt update libignition-math4
        
        sudo apt-get install ariac3
        
- Install ROS MoveIt:
        
        sudo apt-get install ros-melodic-moveit
        
- Source the ROS setup script.
- Create a new catkin workspace.
- Clone the ecse_373_ariac package into the src directory of your workspace:

        git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
        
- Clone this repository into the src directory of your workspace:

        git clone https://github.com/mac411/ecse373_f21_team7_lab_comp.git
        
- Build the packages with catkin and source the devel setup script.

## Running the Node
There are two launch files included in this package that must be launched sequantially. One launch file with start the competition environment, and the other will start the competition. The process is described below:  
- Launch the competition environment:
        
        roslaunch lab_comp competition_environment.launch
- The image below shows what the environment should look like 
![alt text](https://github.com/mac411/ecse373_f21_team7_lab_comp/blob/master/Screenshot%20from%202021-11-09%2017-23-44.png)
- Wait for the GUI to finish loading and the messages in the Terminal to stop updating. Now the environment is fully set up and you are ready to start the competition. Use the following command:

        roslaunch lab_comp lab_comp.launch
        
-Once launched the robot arm will be seen moving over box 4 this is an addition that happened after lab 5. The arm will look over each of the tools in the box and move accordingly. The image below shows what the arm moving in the envirment should look like. 
![alt text](https://github.com/mac411/ecse373_f21_team7_lab_comp/blob/master/Screenshot%20from%202021-11-09%2017-24-49.png)        
### Note
The simulation environment process must be killed and restarted manually in order start the competition over again. The node will warn you if it has not executed properly.
