#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
from moveit_commander.conversions import pose_to_list
import csv_write
from math import pi

#########################   Node Initialization     #########################

rospy.init_node('panel_traverse',anonymous=True)

#########################   Moveit Initialization   #########################

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.robot.RobotCommander()
move_group = moveit_commander.MoveGroupCommander('manipulator')
gripper = moveit_commander.MoveGroupCommander('gripper')

move_group.set_planning_time(5)
move_group.allow_replanning(True)

move_group.set_pose_reference_frame('base_link')

move_group.set_goal_position_tolerance(0.005)
move_group.set_goal_orientation_tolerance(0.1)

move_group.set_planner_id("APSkConfigDefault") # perfect

move_group.set_end_effector_link('tool0')
end_effector_link = move_group.get_end_effector_link()

display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

#########################   Moving Functions  #########################

def move_rotational(joint_goal) : 
  
  move_group.go(joint_goal, wait=True)

#*********************************#

def move_right(x):
  
  curr_pos=move_group.get_current_joint_values()  
  curr_pos[0]-= (x)
  move_rotational(curr_pos)

#*********************************#

def move_left(x):
  
  curr_pos=move_group.get_current_joint_values()  
  curr_pos[0] += (x)
  move_rotational(curr_pos) 

#*********************************#

def move_down(x):
  
  curr_pos=move_group.get_current_joint_values()  
  curr_pos[2] += (x)
  move_rotational(curr_pos)  

#*********************************#

def move_up(x):
  
  curr_pos=move_group.get_current_joint_values()  
  curr_pos[2] -= (x)
  move_rotational(curr_pos) 

#########################   Specific Aruco Functions  #########################

    
def panel_traverse():

  move_left(0.25)  
  move_up(0.25)
  move_right(0.25)
  move_right(0.25)
  move_right(0.25)

  move_down(0.25)
  move_left(0.25)
  move_left(0.25)
  move_left(0.25)

  move_down(0.25)
  move_right(0.25)
  move_right(0.25)
  move_right(0.25)

#*********************************#

def detect_aruco_14():
  
  pose=[-1.25, -pi/2, 1.74, 1.35, pi/2, -pi/2]
  move_rotational(pose)
  move_right(0.15)
  move_up(0.3)
  move_right(0.3)

#*********************************#

def detect_aruco_10():

  pose = [0.8, -pi/2, 1.74, 1.35, pi/2, -pi/2]
  move_rotational(pose)
  move_up(0.15)
  move_right(0.3)
  move_up(0.15)
  move_right(0.3)
 
#*********************************#

def detect_aruco_13():

  pose=[-1, -2.1, 2, 0.35, pi/2, -pi/2]
  move_rotational(pose)
  move_left(0.15)
  move_up(0.15)
  move_right(0.3)
  move_down(0.15)
  move_left(0.15)
#*********************************#

def detect_aruco_11():
  
  pose = [0.4, -2*pi/3 -0.15,5*pi/9 ,pi/9 ,pi/2 ,-pi/2]
  move_rotational(pose)
  move_left(0.15)
  move_down(0.15)
  move_right(0.3)
  move_up(0.15)
  move_left(0.15)
#*********************************#

if __name__ == '__main__':

  try :
    
    init_pos=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
    move_rotational(init_pos) 

    panel_traverse()
    rospy.sleep(2)
    move_rotational(init_pos) 
    detect_aruco_11()
    rospy.sleep(2)
    detect_aruco_10()
    rospy.sleep(2)
    detect_aruco_14()
    rospy.sleep(2)
    detect_aruco_13()
    rospy.sleep(2)

    move_rotational(init_pos) 
  
  except rospy.ROSInterruptException:
    pass

        