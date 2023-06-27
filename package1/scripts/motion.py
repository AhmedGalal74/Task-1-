#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

def callback(msg):
    
    movm = Twist()
    movm1 = Twist()
    movm.linear.x = msg.buttons[1]
    movm.linear.y = msg.buttons[4]
    movm.linear.z = msg.buttons[0]
    movm1.linear.x = msg.buttons[3]
    movm1.linear.y = msg.buttons[5]
    movm1.linear.z = msg.buttons[2]

    if movm.linear.x == 1:
        pub.publish(1)
    elif movm.linear.y == 1:
        pub.publish(2)
    elif movm.linear.z == 1:
        pub.publish(3)
    elif movm1.linear.x == 1:
        pub.publish(4)
    elif movm1.linear.y == 1:
        pub.publish(5)
    elif movm1.linear.z == 1:
        pub.publish(6)


if __name__ == "__main__":

        try:
            rospy.init_node("joy_motion",anonymous = True)
            rospy.Subscriber('joy',Joy,callback)
            pub = rospy.Publisher("motion55" ,Int16 ,queue_size = 10)
            rospy.spin()

        except rospy.ROSInterruptException:
            pass
