#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg as msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from math import pi

target = PoseStamped()
target.header.frame_id = "base_link"
pub1 = rospy.Publisher("/gripper_command",String,queue_size=10)

def press_sw(n):
    if n == 1:
        target.pose.position.x = 0.47383634207163633 -0.25 + 0.015
        target.pose.position.y = 0.06049274094649211
        target.pose.position.z = 0.4054761505686745 -(0.025 + 0.03)
    elif n == 2:
        target.pose.position.x = 0.47561404739477164 -0.25 + 0.015
        target.pose.position.y = -0.05905994551971318
        target.pose.position.z = 0.40561617106851344 -(0.025 + 0.03)
    elif n == 3:
        target.pose.position.x = 0.47440317537918986 -0.25 + 0.015
        target.pose.position.y = 0.06058869081577471
        target.pose.position.z = 0.22509202585329266 -(0.025 + 0.03)
    elif n == 4:
        target.pose.position.x = 0.47357900113993956 -0.25 + 0.015
        target.pose.position.y = -0.05953797097700723
        target.pose.position.z = 0.22509202585329266 -(0.025 + 0.03)

    (plan,_)= moveit_gp.compute_cartesian_path([target.pose],0.01,0)
    moveit_gp.execute(plan, wait=True)
    pub1.publish("close")
    rospy.sleep(2)


rospy.init_node("moveit_press_node", anonymous = True)

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.robot.RobotCommander()

moveit_gp = moveit_commander.MoveGroupCommander("manipulator")
hand = moveit_commander.MoveGroupCommander("gripper")

moveit_gp.set_pose_reference_frame("base_link")
moveit_gp.set_end_effector_link("tool0")
moveit_gp.set_goal_position_tolerance(0.005)
moveit_gp.allow_replanning(True)

pub = rospy.Publisher("/move_group/display_planned_path",msg.DisplayTrajectory,queue_size=20)

home = [0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
moveit_gp.go(home, wait=True)

end_effector_link = "tool0"
start_pos = moveit_gp.get_current_pose(end_effector_link).pose

target.pose.orientation.x = start_pos.orientation.x
target.pose.orientation.y = start_pos.orientation.y
target.pose.orientation.z = start_pos.orientation.z
target.pose.orientation.w = start_pos.orientation.w

print(moveit_gp.get_active_joints())

print("Home Position now")

list1 =[]
for i in range (4):
    x = input("Enter the order of switches, please: ")
    list1.append(x)
print(list1)

for i in list1 :
    press_sw(int(i))
    rospy.sleep(5)
    print(f"switch {i} pressed successfully")


moveit_gp.go(home, wait= True)
print("Done and returned Home successfully")
pub1.publish("open")


#moveit_gp.set_start_state(robot.get_current_state())
#moveit_gp.set_pose_target(target, end_effector_link)
#success = moveit_gp.go(wait=True)


'''
sw1
  position: 
    x: 0.47383634207163633
    y: 0.06049274094649211
    z: 0.4054761505686745
  orientation: 
    x: 0.49930935703046364
    y: -0.49916088696513583
    z: -0.5009046296894403
    w: 0.5006227390577548

'''
'''
sw2
pose: 
pose: 
  position: 
    x: 0.47561404739477164
    y: -0.05905994551971318
    z: 0.40561617106851344
  orientation: 
    x: 0.4870965688957353
    y: -0.4285295875809234
    z: -0.516099544919249
    w: 0.5592321386256962
'''
'''
sw3
  position: 
    x: 0.47440317537918986
    y: 0.06058869081577471
    z: 0.22509202585329266
  orientation: 
    x: 0.4962528833591765
    y: -0.4962449383001106
    z: -0.5042429305438841
    w: 0.5032028457448734
'''
'''
sw4
  position: 
    x: 0.47357900113993956
    y: -0.05953797097700723
    z: 0.22563163025571545
  orientation: 
    x: 0.4995335152555258
    y: -0.49874444417433844
    z: -0.5009049342420916
    w: 0.5008138310725991

'''