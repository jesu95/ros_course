#include "ros/ros.h"
#include "ros_service_assignment/RectangleAreaService.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectangle_area_client");
  if (argc != 3)
  {
    ROS_INFO("usage: rectangle_area_client b h");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_service_assignment::RectangleAreaService>("rectangle_area");
  ros_service_assignment::RectangleAreaService srv;
  srv.request.b = atoll(argv[1]);
  srv.request.h = atoll(argv[2]);
  
  if (client.call(srv))
  {
    ROS_INFO("Area: %ld", (long int)srv.response.area);
  }
  else
  {
    ROS_ERROR("Failed to call service rectangle_area_client");
    return 1;
  }

  return 0;
}