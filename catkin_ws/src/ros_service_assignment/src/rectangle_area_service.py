#!/usr/bin/env python

from __future__ import print_function

import rospy
from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse

def service_handle(req):
    area = req.width*req.height
    print("Handling Service Call")
    print("Desired Width:", req.width, ", Desired Height:", req.height)
    print("Resulting area (A = ", area, ") returned to client",sep="")
    return RectangleAreaServiceResponse(area)

def run_server():
    rospy.init_node('get_rect_area_server')
    print("Initializing Server")
    s = rospy.Service('get_rect_area', RectangleAreaService, service_handle)
    rospy.spin()

if __name__ == '__main__':
    run_server()