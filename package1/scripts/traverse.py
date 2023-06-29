#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import csv_write
import moveit_msgs.msg as msg
from geometry_msgs.msg import PoseStamped
from math import pi
from csv_read import get_pos

rospy.init_node("moveit_press_node", anonymous = True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.robot.RobotCommander()
moveit_gp = moveit_commander.MoveGroupCommander("manipulator")
hand = moveit_commander.MoveGroupCommander("gripper")

moveit_gp.set_pose_reference_frame("base_link")
moveit_gp.set_end_effector_link("tool0")
moveit_gp.set_goal_position_tolerance(0.005)
moveit_gp.set_planning_time(5)
moveit_gp.allow_replanning(True)
pub = rospy.Publisher("/move_group/display_planned_path",msg.DisplayTrajectory,queue_size=20)

def go_delay(pos):
        moveit_gp.go(pos, wait=True)
        rospy.sleep(2)
        
def Traverse ():
        print ("#############starting###########")
        pos1= [0.10007249103361762, -2.094304532481178, 1.789424605603017, 0.34880856674207017, 1.5707935270862619, -1.570854054406988]
        go_delay(pos1)

        pos1= [0.5000258595991278, -1.6944622357838153, 1.8893902132858753, 0.6487277358514777, 1.6707369736010054, -1.5707083676273177]
        go_delay(pos1)

        pos1= [-0.4999741404008722, -1.6944622357838153, 1.8893902132858753, 0.6487277358514777, 1.6707369736010054, -1.5707083676273177]
        go_delay(pos1)
        
        pos1= [-0.09992750896638239, -2.4943045324811783, 1.789424605603017, 0.34880856674207017, 1.5707935270862619, -1.570854054406988]
        go_delay(pos1)
        pos1= [-0.09992750896638239, -2.4943045324811783, 2.1894246056030173, 0.34880856674207017, 1.670793527086262, -1.570854054406988]
        go_delay(pos1)
        
        pos1=[-0.5999613598024223, -2.294563788670477, 1.7207320214606074, 0.6502522780386224, 1.670787729708466, -1.570661938820356]
        go_delay(pos1)

        pos1= [0.2000258595991278, -2.394462235783816, 1.489390213285875, 0.6487277358514777, 1.6707369736010054, -1.5707083676273177]
        go_delay(pos1)

        pos1= [0.40006882077927997, -2.394547891831385, 1.5207773353134089, 0.6501786989873253, 1.670682515120844, -1.570795821464654]
        go_delay(pos1)
        print ("############finisihing###########")

if __name__ == "__main__":
    try:
        home = [0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
        moveit_gp.go(home, wait=True)
        Traverse()
        moveit_gp.go(home, wait=True)
        print("Task 1 Finsished")
        
    except rospy.ROSInterruptException:
        pass

