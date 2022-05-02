#include "ros/ros.h"
#include "ros_service_assignment/RectangleAreaService.h"

bool add(ros_service_assignment::RectangleAreaService::Request  &req,
         ros_service_assignment::RectangleAreaService::Response &res)
{
  res.area = req.b * req.h;
  ROS_INFO("request rectangle area of: b=%ld, h=%ld", (long int)req.b, (long int)req.h);
  ROS_INFO("returning area: [%ld]", (long int)res.area);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectangle_area_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("rectangle_area", add);
  ROS_INFO("Ready to calculate the area of a rectangle.");
  ros::spin();

  return 0;
}