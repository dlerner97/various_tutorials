#!/usr/bin/env python

from __future__ import print_function

import rospy
from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse

def run_client(width, height):
    rospy.wait_for_service('get_rect_area')
    print("Sending service request...")

    try:
        get_rect_area = rospy.ServiceProxy('get_rect_area', RectangleAreaService)
        resp = get_rect_area(width,height)
        print("Rect Area, A =", resp)

        return resp.area
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node("rectagnle_area_service_client")
    run_client(3, 6)