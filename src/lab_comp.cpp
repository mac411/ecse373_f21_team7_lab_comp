#include <string>
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "ros/types.h"
#include "std_msgs/String.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/GetMaterialLocations.h"
#include <osrf_gear/GetMaterialLocationsRequest.h>
#include <osrf_gear/GetMaterialLocationsResponse.h>
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf2_ros/transform_listener.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"   
#include "geometry_msgs/TransformStamped.h" 
tf2_ros::Buffer   tfBuffer; 
std_srvs::Trigger begin_comp; 
int service_call_succeeded;
geometry_msgs::TransformStamped   tfStamped;   
std::vector<osrf_gear::Order> order_vector;
osrf_gear::GetMaterialLocations material_location;
int location_call_succeeded;
std_msgs::String object_type;
osrf_gear::StorageUnit location;
std::vector<osrf_gear::LogicalCameraImage> image_vector(10);
sensor_msgs::JointState joint_states;
std::vector<osrf_gear::Model> desired;
trajectory_msgs::JointTrajectory joint_trajectory;
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6]; 

void orderCallback(const osrf_gear::Order::ConstPtr& order)
{
	osrf_gear::Order o;
	o.order_id = order->order_id;
	o.shipments = order->shipments;
	order_vector.push_back(o);
}

void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& image, int index)
{
	osrf_gear::LogicalCameraImage i;
	i.models = image->models;
	i.pose = image->pose;
	image_vector[index] = i;
}

void bin1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,0);
}
void bin2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,1);
}
void bin3Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,2);
}
void bin4Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,3);
}
void bin5Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,4);
}
void bin6Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,5);
}

void agv1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,6);
}

void agv2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,7);
}

void qc1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,8);
}

void qc2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image)
{
	cameraCallback(image,9);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& state)
{
    joint_states.effort = state->effort;
	joint_states.header = state->header;
	joint_states.name = state->name;
	joint_states.position = state->position;
	joint_states.velocity = state->velocity;
}

std::string pose2str(geometry_msgs::Pose pose)
{
	std::string posestring = "Position: x="+std::to_string(pose.position.x)+" y="+std::to_string(pose.position.y)+" z="+std::to_string(pose.position.z)+" Orientation: x="+std::to_string(pose.orientation.x)+" y="+std::to_string(pose.orientation.y)+" z="+std::to_string(pose.orientation.z)+" w="+std::to_string(pose.orientation.w);
	return posestring;
}

int main(int argc, char **argv)
{
	order_vector.clear();
	image_vector.clear();
   
  	ros::init(argc, argv, "lab_comp");
 	ros::NodeHandle n;
	tf2_ros::TransformListener    tfListener(tfBuffer) ;   
 	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
  	ros::Subscriber order_subscriber = n.subscribe("ariac/orders",1000,orderCallback);
	ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("ariac/material_locations");
  	ros::Subscriber bin1_sub = n.subscribe("ariac/logical_camera_bin1",1,bin1Callback);
	ros::Subscriber bin2_sub = n.subscribe("ariac/logical_camera_bin2",1,bin2Callback);
  	ros::Subscriber bin3_sub = n.subscribe("ariac/logical_camera_bin3",1,bin3Callback);
  	ros::Subscriber bin4_sub = n.subscribe("ariac/logical_camera_bin4",1,bin4Callback);
  	ros::Subscriber bin5_sub = n.subscribe("ariac/logical_camera_bin5",1,bin5Callback);
  	ros::Subscriber bin6_sub = n.subscribe("ariac/logical_camera_bin6",1,bin6Callback);
	ros::Subscriber agv1_sub = n.subscribe("ariac/logical_camera_agv1",1,agv1Callback);
	ros::Subscriber agv2_sub = n.subscribe("ariac/logical_camera_agv2",1,agv2Callback);
  	ros::Subscriber qc1_sub = n.subscribe("ariac/quality_control_sensor_1",1,qc1Callback);
  	ros::Subscriber qc2_sub = n.subscribe("ariac/quality_control_sensor_2",1,qc2Callback);
	ros::Subscriber joint_state_sub = n.subscribe("ariac/arm1/joint_states",10,jointStateCallback);
    ros::Publisher joint_state_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command",1000);

	service_call_succeeded = begin_client.call(begin_comp);
	if(!service_call_succeeded) {
		ROS_ERROR("Competition service call failed! Goodness Gracious!!");
	}
	else {
		if(begin_comp.response.success) {
			ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
		}
		else {
			ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
		}
	}

	ros::Rate loop_rate(1);

	int count = 0;
	int model_num = 0;
  	while(ros::ok())
	{
		loop_rate.sleep();
		if(order_vector.size() > 0)
		{
			material_location.request.material_type = order_vector.front().shipments.front().products.front().type;
			location_call_succeeded = material_client.call(material_location);
			ROS_INFO("The object is of type: %s", material_location.request.material_type.c_str());
			ROS_INFO("The storage unit containing this object is: %s", material_location.response.storage_units.front().unit_id.c_str());
			
			try    {   
				tfStamped   =   tfBuffer.lookupTransform( "arm1_base_link" ,  "logical_camera_bin4_frame" ,   
				ros::Time( 0.0 ),   ros::Duration( 1.0 ));   
				ROS_DEBUG( "Transform   to   [%s]   from   [%s]" ,   tfStamped.header.frame_id.c_str(),   
				tfStamped.child_frame_id.c_str());   
			}catch    (tf2::TransformException   &ex)   {   
				ROS_ERROR( "%s" ,   ex.what());   
			}   
						order_vector.clear();
			std::string product_type = material_location.request.material_type;
			for(int i = 0; i < 6; i++)
			{
				for(int j = 0; j < image_vector[i].models.size(); j++)
				{
					if(image_vector[i].models[j].type == product_type)
					{
						desired.push_back(image_vector[i].models[j]);
						ROS_INFO("Product type: %s, Bin: %s, Pose: %s", product_type.c_str(), std::to_string(i).c_str(), pose2str(image_vector[i].models[j].pose).c_str());
					}
				}
			}
		}
		if (model_num < desired.size())
		{
			loop_rate.sleep();
			geometry_msgs::PoseStamped   part_pose,   goal_pose;
			part_pose.pose = desired[model_num].pose;
			tf2::doTransform(part_pose,   goal_pose,   tfStamped);  
			q_pose[0] = joint_states.position[1];   
			q_pose[1] = joint_states.position[2];   
			q_pose[2] = joint_states.position[3];   
			q_pose[3] = joint_states.position[4];   
			q_pose[4] = joint_states.position[5];   
			q_pose[5] = joint_states.position[6];

			ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
			
			T_des[0][3] = goal_pose.pose.position.x;   
			T_des[1][3] = goal_pose.pose.position.y;   
			T_des[2][3] = goal_pose.pose.position.z + 0.3;
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
			joint_trajectory.points[1].positions[0] = joint_states.position[1];
			for (int indy = 0; indy < 6; indy++)
			{
				joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
			}
			joint_trajectory.points[1].time_from_start = ros::Duration(1.0); 

			joint_state_pub.publish(joint_trajectory);
			model_num++;
			loop_rate.sleep();
		}
		ros::spinOnce();
	}
  	return 0;
}