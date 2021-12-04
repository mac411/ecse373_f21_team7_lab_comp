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
#include "osrf_gear/VacuumGripperControl.h"
#include "angles/angles.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/AGVControl.h"
#include "std_msgs/Float32.h"

tf2_ros::Buffer tfBuffer;
std_srvs::Trigger begin_comp;
int service_call_succeeded;
geometry_msgs::TransformStamped tfStamped;
std::vector<osrf_gear::Order> order_vector;
osrf_gear::GetMaterialLocations material_location;
int location_call_succeeded;
std_msgs::String object_type;
osrf_gear::StorageUnit location;
std::vector<osrf_gear::LogicalCameraImage> image_vector(10);
sensor_msgs::JointState joint_states;
std::vector<osrf_gear::Model> desired;
std::vector<geometry_msgs::Pose> tray_poses;
std::vector<std::__cxx11::string> agv_ids;
trajectory_msgs::JointTrajectory joint_trajectory;
osrf_gear::VacuumGripperControl gripper_control;
osrf_gear::VacuumGripperState gripper_state;
sensor_msgs::JointState prev_joint_state;
std::vector<osrf_gear::Shipment> shipment_vec;
std::vector<int> bin_numbers;
std::vector<std::string> shipment_types;
osrf_gear::AGVControl agv1_srv;
osrf_gear::AGVControl agv2_srv;
bool agv1_resp;
bool agv2_resp;
float score;
std::string agv1_state;
std::string agv2_state;
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];
int count = 0;

void orderCallback(const osrf_gear::Order::ConstPtr &order)
{
	osrf_gear::Order o;
	o.order_id = order->order_id;
	o.shipments = order->shipments;
	order_vector.push_back(o);
}

void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &image, int index)
{
	osrf_gear::LogicalCameraImage i;
	i.models = image->models;
	i.pose = image->pose;
	image_vector[index] = i;
}

void bin1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 0);
}
void bin2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 1);
}
void bin3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 2);
}
void bin4Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 3);
}
void bin5Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 4);
}
void bin6Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 5);
}

void agv1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 6);
}

void agv2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 7);
}

void qc1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 8);
}

void qc2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image)
{
	cameraCallback(image, 9);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &state)
{
	joint_states.effort = state->effort;
	joint_states.header = state->header;
	joint_states.name = state->name;
	joint_states.position = state->position;
	joint_states.velocity = state->velocity;
}

void gripperCallback(const osrf_gear::VacuumGripperStateConstPtr &state)
{
	gripper_state.attached = state->attached;
	gripper_state.enabled = state->enabled;
}

std::string pose2str(geometry_msgs::Pose pose)
{
	std::string posestring = "Position: x=" + std::to_string(pose.position.x) + " y=" + std::to_string(pose.position.y) + " z=" + std::to_string(pose.position.z) + " Orientation: x=" + std::to_string(pose.orientation.x) + " y=" + std::to_string(pose.orientation.y) + " z=" + std::to_string(pose.orientation.z) + " w=" + std::to_string(pose.orientation.w);
	return posestring;
}

void scoreCallback(const std_msgs::Float32::ConstPtr &latest_score)
{
	score = latest_score->data;
}

void agv1StateCallback(const std_msgs::String::ConstPtr &state)
{
	agv1_state = state->data;
}

void agv2StateCallback(const std_msgs::String::ConstPtr &state)
{
	agv2_state = state->data;
}

trajectory_msgs::JointTrajectory generate_traj(geometry_msgs::Pose desired_pose, double lin_act)
{
	// desired_pose should be desired[model_num].pose for a part
	geometry_msgs::PoseStamped des_pose, goal_pose;
	des_pose.pose = desired_pose;
	tf2::doTransform(des_pose, goal_pose, tfStamped);
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];

	ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

	goal_pose.pose.orientation.w = 0.707;
	goal_pose.pose.orientation.x = 0;
	goal_pose.pose.orientation.y = 0.707;
	goal_pose.pose.orientation.z = 0;
	T_des[0][3] = goal_pose.pose.position.x;
	T_des[1][3] = goal_pose.pose.position.y;
	T_des[2][3] = goal_pose.pose.position.z;
	T_des[3][3] = 1.0;

	T_des[0][0] = 0.0;
	T_des[0][1] = -1.0;
	T_des[0][2] = 0.0;
	T_des[1][0] = 0.0;
	T_des[1][1] = 0.0;
	T_des[1][2] = 1.0;
	T_des[2][0] = -1.0;
	T_des[2][1] = 0.0;
	T_des[2][2] = 0.0;
	T_des[3][0] = 0.0;
	T_des[3][1] = 0.0;
	T_des[3][2] = 0.0;

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
	for (int i = 0; i < num_sols; i++)
	{
		q_des_indx = i;
		if (q_des[q_des_indx][1] >= 3 * 1.5707 && q_des[q_des_indx][1] <= 4 * 1.5707)
		{
			if (abs(q_des[q_des_indx][4] - (3 * 1.5707)) < 0.5)
			{
				break;
			}
		}
	}

	q_des[q_des_indx][1] = angles::normalize_angle(q_des[q_des_indx][1]);

	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	joint_trajectory.points[1].positions[0] = lin_act;
	for (int indy = 0; indy < 6; indy++)
	{
		joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
	}
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

	return joint_trajectory;
}

trajectory_msgs::JointTrajectory linear_act(double lin_act)
{
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];
	//q_pose[1] = angles::normalize_angle(q_pose[1]);

	ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

	T_des[0][3] = T_pose[0][3];
	T_des[1][3] = T_pose[1][3];
	T_des[2][3] = T_pose[2][3];
	T_des[3][3] = 1.0;

	T_des[0][0] = 0.0;
	T_des[0][1] = -1.0;
	T_des[0][2] = 0.0;
	T_des[1][0] = 0.0;
	T_des[1][1] = 0.0;
	T_des[1][2] = 1.0;
	T_des[2][0] = -1.0;
	T_des[2][1] = 0.0;
	T_des[2][2] = 0.0;
	T_des[3][0] = 0.0;
	T_des[3][1] = 0.0;
	T_des[3][2] = 0.0;

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
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
	{
		for (int indz = 0; indz < joint_states.name.size(); indz++)
		{
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
			{
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}

	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

	joint_trajectory.points[1].positions[0] = lin_act;

	return joint_trajectory;
}

int main(int argc, char **argv)
{
	order_vector.clear();
	image_vector.clear();

	ros::init(argc, argv, "lab_comp");
	ros::NodeHandle n;
	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	ros::Subscriber order_subscriber = n.subscribe("ariac/orders", 1000, orderCallback);
	ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("ariac/material_locations");
	ros::Subscriber bin1_sub = n.subscribe("ariac/logical_camera_bin1", 1, bin1Callback);
	ros::Subscriber bin2_sub = n.subscribe("ariac/logical_camera_bin2", 1, bin2Callback);
	ros::Subscriber bin3_sub = n.subscribe("ariac/logical_camera_bin3", 1, bin3Callback);
	ros::Subscriber bin4_sub = n.subscribe("ariac/logical_camera_bin4", 1, bin4Callback);
	ros::Subscriber bin5_sub = n.subscribe("ariac/logical_camera_bin5", 1, bin5Callback);
	ros::Subscriber bin6_sub = n.subscribe("ariac/logical_camera_bin6", 1, bin6Callback);
	ros::Subscriber agv1_sub = n.subscribe("ariac/logical_camera_agv1", 1, agv1Callback);
	ros::Subscriber agv2_sub = n.subscribe("ariac/logical_camera_agv2", 1, agv2Callback);
	ros::Subscriber qc1_sub = n.subscribe("ariac/quality_control_sensor_1", 1, qc1Callback);
	ros::Subscriber qc2_sub = n.subscribe("ariac/quality_control_sensor_2", 1, qc2Callback);
	ros::Subscriber joint_state_sub = n.subscribe("ariac/arm1/joint_states", 10, jointStateCallback);
	ros::Publisher joint_state_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 1000);
	ros::ServiceClient gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("ariac/arm1/gripper/control");
	ros::Subscriber gripper_sub = n.subscribe("ariac/arm1/gripper/state", 1, gripperCallback);
	ros::ServiceClient agv1_client = n.serviceClient<osrf_gear::AGVControlRequest>("ariac/agv1");
	ros::ServiceClient agv2_client = n.serviceClient<osrf_gear::AGVControlRequest>("ariac/agv2");
	ros::Subscriber score_sub = n.subscribe("ariac/current_score",1,scoreCallback);
	ros::Subscriber agv1_state_sub = n.subscribe("ariac/agv1",1,agv1StateCallback);
	ros::Subscriber agv2_state_sub = n.subscribe("ariac/agv2",1,agv2StateCallback);

	service_call_succeeded = begin_client.call(begin_comp);
	if (!service_call_succeeded)
	{
		ROS_ERROR("Competition service call failed! Goodness Gracious!!");
	}
	else
	{
		if (begin_comp.response.success)
		{
			ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
		}
		else
		{
			ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
		}
	}

	ros::Rate loop_rate(0.5);

	int order_idx = 0;
	int order_num = 0;
	bool order_fulfilled = false;
	int shipment1 = 0;
	int shipment2 = 0;
	while (ros::ok())
	{
		ROS_INFO("Current score is %s", std::to_string(score).c_str());
		loop_rate.sleep();
		if (order_vector.size() > 0 && order_num < order_vector.size())
		{
			order_fulfilled = false;
			desired.clear();
			tray_poses.clear();
			agv_ids.clear();
			bin_numbers.clear();
			shipment_types.clear();
			order_num += 1;
			std::vector<int> last_bin_models;
			last_bin_models.resize(6);
			for (int shipment_ind = 0; shipment_ind < order_vector[order_idx].shipments.size(); shipment_ind++)
			{
				for (int product_ind = 0; product_ind < order_vector[order_idx].shipments[shipment_ind].products.size(); product_ind++)
				{
					std::string type = order_vector[order_idx].shipments[shipment_ind].products[product_ind].type;
					for (int i = 0; i < 6; i++)
					{
						for (int j = 0; j < image_vector[i].models.size(); j++)
						{
							if (image_vector[i].models[j].type == type && j >= last_bin_models[i])
							{
								ROS_INFO("Product %s found", std::to_string(product_ind + 1).c_str());
								ROS_INFO("Product type %s", type.c_str());
								ROS_INFO("Bin number %s", std::to_string(i + 1).c_str());
								ROS_INFO("AGV number %s", order_vector[order_idx].shipments[shipment_ind].agv_id.c_str());
								ROS_INFO("Shipment type %s", order_vector[order_idx].shipments[shipment_ind].shipment_type.c_str());
								desired.push_back(image_vector[i].models[j]);
								tray_poses.push_back(order_vector[order_idx].shipments[shipment_ind].products[product_ind].pose);
								agv_ids.push_back(order_vector[order_idx].shipments[shipment_ind].agv_id);
								bin_numbers.push_back(i + 1);
								last_bin_models[i] = j + 1;
								break;
							}
						}
					}
				}
				shipment_types.push_back(order_vector[order_idx].shipments[shipment_ind].shipment_type);
			}
		}
		int model_num = 0;
		while (!order_fulfilled && desired.size() > 0)
		{

			ROS_INFO("Arm loop");

			if (model_num < desired.size())
			{
				if (model_num != 0)
				{
					joint_states = prev_joint_state;
				}
				loop_rate.sleep();
				std::string bin_frame = "logical_camera_bin" + std::to_string(bin_numbers[model_num]) + "_frame";
				try
				{
					tfStamped = tfBuffer.lookupTransform("world", bin_frame,
														 ros::Time(0.0), ros::Duration(1.0));
					ROS_INFO("Transform   to   [%s]   from   [%s]", tfStamped.header.frame_id.c_str(),
							 tfStamped.child_frame_id.c_str());
				}
				catch (tf2::TransformException &ex)
				{
					ROS_ERROR("%s", ex.what());
				}
				geometry_msgs::Pose des = desired[model_num].pose;
				geometry_msgs::Pose des_tf;
				tf2::doTransform(des, des_tf, tfStamped);
				double offset = 0.8;
				double correction = 0.0;
				if(des.position.y <= 0.05)
				{
					correction = 0.45;
				}
				else
				{
					correction = 0.2;
				}
				double lin_act = des_tf.position.y - offset;
				joint_trajectory = linear_act(lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();
				loop_rate.sleep();

				try
				{
					tfStamped = tfBuffer.lookupTransform("arm1_base_link", bin_frame,
														 ros::Time(0.0), ros::Duration(1.0));
					ROS_INFO("Transform   to   [%s]   from   [%s]", tfStamped.header.frame_id.c_str(),
							 tfStamped.child_frame_id.c_str());
				}
				catch (tf2::TransformException &ex)
				{
					ROS_ERROR("%s", ex.what());
				}

				des.position.x = des.position.x - 0.7;
				des.position.y = des.position.y - offset + correction;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				des.position.x = des.position.x + 0.6;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				des.position.y = des.position.y + offset - correction;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				des.position.x = des.position.x + 0.08;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				gripper_control.request.enable = 1;
				gripper_client.call(gripper_control);
				loop_rate.sleep();

				des.position.x = des.position.x - 0.18;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				des.position.y = des.position.y - offset + correction;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				des.position.x = des.position.x - 0.5;
				joint_trajectory = generate_traj(des, lin_act);
				joint_state_pub.publish(joint_trajectory);
				loop_rate.sleep();

				// Picked up part, now put in agv
				ROS_INFO("%s", agv_ids[model_num].c_str());
				if (strcmp(agv_ids[model_num].c_str(),"agv2") == 0)
				{
					ROS_INFO("Chose agv2");
					ros::spinOnce();
					prev_joint_state = joint_states;
					des.position.y = -2.3;
					joint_trajectory = linear_act(des.position.y);
					joint_state_pub.publish(joint_trajectory);
					loop_rate.sleep();
					loop_rate.sleep();
					try
					{
						tfStamped = tfBuffer.lookupTransform("arm1_base_link", "kit_tray_2",
															 ros::Time(0.0), ros::Duration(1.0));
						ROS_INFO("Transform   to   [%s]   from   [%s]", tfStamped.header.frame_id.c_str(),
								 tfStamped.child_frame_id.c_str());
					}
					catch (tf2::TransformException &ex)
					{
						ROS_ERROR("%s", ex.what());
					}
					des = tray_poses[model_num];
					des.position.z = des.position.z + 0.1;
					joint_trajectory = generate_traj(des, -2.3);
					joint_state_pub.publish(joint_trajectory);
					loop_rate.sleep();
				}
				else
				{
					ROS_INFO("Chose agv1");
					ros::spinOnce();
					prev_joint_state = joint_states;
					des.position.y = 2.3;
					joint_trajectory = linear_act(des.position.y);
					joint_state_pub.publish(joint_trajectory);
					loop_rate.sleep();
					loop_rate.sleep();
					try
					{
						tfStamped = tfBuffer.lookupTransform("arm1_base_link", "kit_tray_1",
															 ros::Time(0.0), ros::Duration(1.0));
						ROS_INFO("Transform   to   [%s]   from   [%s]", tfStamped.header.frame_id.c_str(),
								 tfStamped.child_frame_id.c_str());
					}
					catch (tf2::TransformException &ex)
					{
						ROS_ERROR("%s", ex.what());
					}
					des = tray_poses[model_num];
					des.position.z = des.position.z + 0.1;
					joint_trajectory = generate_traj(des, 2.3);
					joint_state_pub.publish(joint_trajectory);
					loop_rate.sleep();
				}

				gripper_control.request.enable = 0;
				gripper_client.call(gripper_control);
				loop_rate.sleep();

				model_num++;
			}
			else
			{
				order_fulfilled = true;
				order_idx += 1;
				if(shipment_types.size() == 1)
				{
					ROS_INFO("Order 0");
					agv1_srv.request.shipment_type = shipment_types[0];
					agv1_resp = agv1_client.call(agv1_srv);
					ros::Duration(15).sleep();
				}
				else
				{
					if (strcmp(agv_ids[0].c_str(),"agv2") == 0)
					{
						ROS_INFO("Order 1-2");
						agv2_srv.request.shipment_type = shipment_types[0];
						agv1_srv.request.shipment_type = shipment_types[1];
					}
					else
					{
						ROS_INFO("Order 1-1");
						agv1_srv.request.shipment_type = shipment_types[0];
						agv2_srv.request.shipment_type = shipment_types[1];
					}
					agv1_resp = agv1_client.call(agv1_srv);
					agv2_resp = agv2_client.call(agv2_srv);
					ros::Duration(15).sleep();
				}
			}
			ros::spinOnce();
		}
		ros::spinOnce();
	}
	return 0;
}