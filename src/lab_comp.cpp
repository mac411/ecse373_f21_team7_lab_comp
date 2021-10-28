#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "ros/types.h"
#include "std_msgs/String.h"
#include <vector>
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "geometry_msgs/Pose.h"
#include "osrf_gear/GetMaterialLocations.h"
#include <osrf_gear/GetMaterialLocationsRequest.h>
#include <osrf_gear/GetMaterialLocationsResponse.h>
#include "osrf_gear/StorageUnit.h"

std_srvs::Trigger begin_comp; 
int service_call_succeeded;
std::vector<osrf_gear::Order> order_vector;
osrf_gear::GetMaterialLocations material_location;
int location_call_succeeded;
std_msgs::String object_type;
osrf_gear::StorageUnit location;

void orderCallback(const osrf_gear::Order order)
{
	order_vector.push_back(order);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "lab_comp");

  ros::NodeHandle n;
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
  ros::Subscriber order_subscriber = n.subscribe("ariac/orders",1000,orderCallback);
	ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("ariac/material_locations");
  
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
	
	material_location.request.material_type = order_vector.front().shipments.front().shipment_type;
	location_call_succeeded = material_client.call(material_location);
	ROS_INFO("The object is of type: %s", material_location.request.material_type);
	ROS_INFO("The storage unit containing this object is: %s", material_location.response.storage_units);

  ros::spin();

  return 0;
}
