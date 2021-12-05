# ecse373_f21_team7_lab_comp
This README will describe how to use the `lab_comp` ROS package. Please note that this repository contains several tags indicating separate phases of development and functionality. They will be described below, but the method of running the node remains constant across all phases.

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
There are two launch files included in this package that must be launched sequentially. One launch file is used to start the competition environment, and the other is used to run the node from this package. The process is described below:  
- Launch the Gazebo competition environment (in the background):
        
        roslaunch lab_comp competition_environment.launch &
- The image below shows what the environment should look like:  

![alt text](https://github.com/mac411/ecse373_f21_team7_lab_comp/blob/master/Screenshot%20from%202021-11-09%2017-23-44.png)
- Wait for the GUI to finish loading and the messages in the terminal to stop updating. Now, the competition environment is fully set up and ready to receive instruction from our node. Use the following command:

        roslaunch lab_comp lab_comp.launch
        
<!-- - Once launched the robot arm will be seen moving over box 4 this is an addition that happened after lab 5. The arm will look over each of the tools in the box and move accordingly. The image below shows what the arm moving in the envirment should look like.  

![alt text](https://github.com/mac411/ecse373_f21_team7_lab_comp/blob/master/Screenshot%20from%202021-11-09%2017-24-49.png) -->  

If all of the commands were executed properly, you should start to see the arm moving in a manner dependent on which Phase of the repository you cloned and are running currently. These modes of operation will be described in the following section.

## Phases

Given that our group fell behind with the weekly Phase updates, they were all completed within a relatively short timespan. As such, the earlier phases lack some of the polish that Phase 4 has because we wanted to put emphasis on having the final Phase functioning properly. Luckily, the vast majority of the previous Phases' functionality is encompassed in Phase 4. So while the previous individual Phases may have some bugs, they were ironed out for the final version.

### Phase 1

This introductory phase tells the arm to grasp and put down each part in a bin one by one.

### Phase 2

This phase uses the previous phase's arm manipulation to pick up a single part and then place it on one of the AGV trays to be submitted.

### Phase 3 (included in Phase2 tag of the repository)

Similar to Phase 2, but now is able to complete an order of two shipments with parts in separate bins. (In actuality, the tag called Phase2 incorporates Phases 2 and 3 because they were so similar. We wrote the code with the final phase in mind, so our Phase 2 code was practically already capable of achieving the Phase 3 specifications).

### Phase 4

Finally, Phase 4 combines all of the previous phases into one that is capable of completing two orders of two shipments of different parts. It is able to achieve almost all the points when being evaluated by the competition environment. You will be able to see the score print out in the terminal after the node ships off the second order.  

## Theory of Operation

This section mostly applies to Phase 4. Below is a block diagram depicting the manner in which our node completes the orders:  

<img width="883" alt="Screen Shot 2021-12-04 at 9 02 07 PM" src="https://user-images.githubusercontent.com/68517385/144730635-80835dcd-c02c-4f4d-b0cc-da6d37a09ea5.png">

## Important Note
The simulation environment process must be killed and restarted manually in order start the competition over again. The node will warn you if it has not executed properly. To kill all the processes, use `^C` to kill our node, followed by the `fg` command to bring the competition environment to the foreground, and then `^C` one more time.
