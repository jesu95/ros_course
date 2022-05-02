#!/usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse

def request_rectangle_area(x, y):
    rospy.wait_for_service('rectangle_area_service')
    try:
        add_two_ints = rospy.ServiceProxy('rectangle_area_service', RectangleAreaService)
        server_response = add_two_ints(x, y)
        return server_response.area
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)

def usage():
    return "%s [x y]", sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print (usage())
        sys.exit(1)
    print ("Requesting the area of a rectangle of width %s and height %s" % (x, y))
    print ("area with width %s and height %s = %s" % (x, y, request_rectangle_area(x, y)))