#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg as msg
from std_msgs.msg import Int16
from math import pi

def callback(msg):
    print("command received")

    if msg.data == 1 :
        points.position.x += 0.02
        print("x inc")
    elif msg.data == 2:
        points.position.y += 0.02
        print("y inc")
    elif msg.data == 3:
        points.position.z += 0.02
        print("z inc")
    elif msg.data == 4:
        points.position.x -= 0.02
        print("x dec")
    elif msg.data == 5:
        points.position.y -= 0.02
        print("y dec")
    elif msg.data == 6:
        points.position.z -= 0.02
        print("z dec")

    (plan,_) = moveit_gp.compute_cartesian_path([points],0.01,0)
    moveit_gp.execute(plan,wait=True)

    print("command executed")

if __name__ =="__main__":

    try:
        rospy.init_node("moveit_node",anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)

        robot=moveit_commander.robot.RobotCommander()
        print(robot.get_group_names())

        moveit_gp = moveit_commander.MoveGroupCommander('manipulator')
        hand= moveit_commander.MoveGroupCommander('gripper')

        moveit_gp.set_pose_reference_frame("base_link")
        moveit_gp.set_end_effector_link("tool0")
        moveit_gp.allow_replanning(True)
        moveit_gp.set_goal_position_tolerance(0.005)

        pub = rospy.Publisher("/move_group/display_planned_path",msg.DisplayTrajectory,queue_size=20)

        home= [0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]

        moveit_gp.go(home, wait=True)
        
        points = moveit_gp.get_current_pose(end_effector_link = "tool0").pose

        rospy.Subscriber("motion55",Int16,callback)
    
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    



