#!/usr/bin/env python

from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse
import rospy

def handle_rectangle_area(req):
    print("Returning area [%s + %s = %s]"%(req.b, req.h, (req.b * req.h)))
    return RectangleAreaServiceResponse(req.b * req.h)

def rectangle_area_server():
    rospy.init_node('rectangle_area_server_node')
    s = rospy.Service('rectangle_area_service', RectangleAreaService, handle_rectangle_area)
    print ("Ready to calculate the area of a rectangle.")
    rospy.spin()
    
if __name__ == "__main__":
    rectangle_area_server()