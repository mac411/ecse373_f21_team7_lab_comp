#include "ros/ros.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

sensor_msgs::JointState joint_states;
trajectory_msgs::JointTrajectory desired;
trajectory_msgs::JointTrajectory joint_trajectory;
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

void jointStateCallback(sensor_msgs::JointState::ConstPtr& state)
{
    joint_states = *state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_arm");

    ros::NodeHandle n;

    ros::Subscriber joint_state_sub = n.subscribe("ariac/joint_states",10,jointStateCallback);
    ros::Publisher joint_state_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm/command",1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok())
	{
        q_pose[0] = joint_states.position[1];   
        q_pose[1] = joint_states.position[2];   
        q_pose[2] = joint_states.position[3];   
        q_pose[3] = joint_states.position[4];   
        q_pose[4] = joint_states.position[5];   
        q_pose[5] = joint_states.position[6];
        ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
	    
        desired.points.

        T_des[0][3] = desired.pose.position.x;   
        T_des[1][3] = desired.pose.position.y;   
        T_des[2][3] = desired.pose.position.z + 0.3;
        T_des[3][3] = 1.0;   
            
        T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;   
        T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;   
        T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;   
        T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;   
        
        int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);  

        joint_trajectory.header.seq = count++;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.header.frame_id = "/world";

        joint_trajectory.joint_names.clear();
        joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");   
        joint_trajectory.joint_names.push_back("shoulder_pan_joint");   
        joint_trajectory.joint_names.push_back("shoulder_lift_joint");   
        joint_trajectory.joint_names.push_back("elbow_joint");   
        joint_trajectory.joint_names.push_back("wrist_1_joint");   
        joint_trajectory.joint_names.push_back("wrist_2_joint");   
        joint_trajectory.joint_names.push_back("wrist_3_joint"); 

        joint_trajectory.points.resize(2);
        joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
        for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
        {
            for (int indz = 0; indz < joint_states.name.size(); indz++)
            {
                if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
                {
                    joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                    break;
                }
            }
        }

        joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

        int q_des_indx = 0;


        joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
        joint_trajectory.points[1].positions[0] = joint_states.position[0];
        for (int indy = 0; indy < 6; indy++)
        {
            joint_trajectory.points[1].positions[indy + 1] = q_sols[q_des_indx][indy];
        }

        loop_rate.sleep();
		ros::spinOnce();
	}
    return 0;
}