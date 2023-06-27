#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Krzysztof Stezala
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Script presenting how to call custom ROS service through Python code

import rospy
from csv_read import get_pos
from erc_aruco_msg.srv import ErcArucoRequest, ErcArucoResponse, ErcAruco
        
class ExampleCall:
    def __init__(self):
        rospy.init_node("erc_aruco_example_caller",anonymous=True)
        rospy.loginfo("Ready to call")
        self.call_aruco_checker()

    def call_aruco_checker(self):
        rospy.wait_for_service("erc_aruco_score")
        try:
            # create service proxy with service name and message type
            service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
            # create object of the request type for the Service (14 tags)
            service_msg = ErcArucoRequest()

            # WARNING!#######################################################################
            #TODO: Below you should specify the tag positions with respect to the /base frame
            service_msg.tag1  = get_pos(1)
            service_msg.tag2  = get_pos(2)
            service_msg.tag3  = get_pos(3)
            service_msg.tag4  = get_pos(4)
            service_msg.tag5  = get_pos(5)
            service_msg.tag6  = get_pos(6)
            service_msg.tag7  = get_pos(7)
            service_msg.tag8  = get_pos(8)
            service_msg.tag9  = get_pos(9)
            service_msg.tag10 = get_pos(10)
            service_msg.tag11 = get_pos(11)
            service_msg.tag12 = get_pos(12)
            service_msg.tag13 = get_pos(13)
            service_msg.tag14 = get_pos(14)
            ##################################################################################

            # call the service with your message through service proxy
            # and receive the response, which happens to be your score
            service_response = service_proxy(service_msg)
            print(f"You received score {service_response.score}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        ex = ExampleCall()
    except KeyboardInterrupt:
        print("end")

