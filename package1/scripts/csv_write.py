#!/usr/bin/env python3
import rospy
import csv
import os
import moveit_msgs.msg as msg
from geometry_msgs.msg import PoseStamped
os.chdir(os.path.dirname(os.path.abspath(__file__)))

flags = [1]*14 

def callback1(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[0] == 1) :
        with open('filex.csv','a') as csv_file:
            flags[0] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 1"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 1 saved successfully")
            rospy.sleep(2)

def callback2(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[1] == 1) :
        with open('filex.csv','a') as csv_file:
            flags[1] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 2"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 2 saved successfully")
            rospy.sleep(2)

def callback3(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[2] == 1) :
        with open('filex.csv','a') as csv_file:
            flags[2] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 3"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 3 saved successfully")
            rospy.sleep(2)

def callback4(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[3] == 1):
        with open('filex.csv','a') as csv_file:
            flags[3] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 4"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 4 saved successfully")
            rospy.sleep(2)

def callback5(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[4] == 1):
        with open('filex.csv','a') as csv_file:
            flags[4] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 5"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 5 saved successfully")
            rospy.sleep(2)

def callback6(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[5] == 1):
        with open('filex.csv','a') as csv_file:
            flags[5] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 6"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 6 saved successfully")
            rospy.sleep(2)

def callback7(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[6] == 1):
        with open('filex.csv','a') as csv_file:
            flags[6] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 7"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 7 saved successfully")
            rospy.sleep(2)

def callback8(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[7] == 1):
        with open('filex.csv','a') as csv_file:
            flags[7] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 8"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 8 saved successfully")
            rospy.sleep(2)

def callback9(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[8] == 1):
        with open('filex.csv','a') as csv_file:
            flags[8] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 9"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 9 saved successfully")
            rospy.sleep(2)

def callback10(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[9] == 1):
        with open('filex.csv','a') as csv_file:
            flags[9] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 10"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 10 saved successfully")
            rospy.sleep(2)

def callback11(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[10] == 1):
        with open('filex.csv','a') as csv_file:
            flags[10] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 11"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 11 saved successfully")
            rospy.sleep(2)

def callback12(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[11] == 1):
        with open('filex.csv','a') as csv_file:
            flags[11] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 12"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 12 saved successfully")
            rospy.sleep(2)

def callback13(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[12] == 1):
        with open('filex.csv','a') as csv_file:
            flags[12] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 13"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 13 saved successfully")
            rospy.sleep(2)

def callback14(msg):
    #target.pose.position.x = msg.pose.position.x
    if (flags[13] == 1):
        with open('filex.csv','a') as csv_file:
            flags[13] = 0
            csv_writer = csv.writer(csv_file,delimiter = ',')
            csv_writer.writerow(["ID = 14"])
            csv_writer.writerow([msg.pose.position])
            print ("ID 14 saved successfully")
            rospy.sleep(2)

def save_pos():
    print("inside save pos")
    sub1 = rospy.Subscriber("/aruco_single1/pose", PoseStamped, callback1)
    sub2 = rospy.Subscriber("/aruco_single2/pose", PoseStamped, callback2)
    sub3 = rospy.Subscriber("/aruco_single3/pose", PoseStamped, callback3)
    sub4 = rospy.Subscriber("/aruco_single4/pose", PoseStamped, callback4)

    sub5 = rospy.Subscriber("/aruco_single5/pose", PoseStamped, callback5)
    sub6 = rospy.Subscriber("/aruco_single6/pose", PoseStamped, callback6)
    sub7 = rospy.Subscriber("/aruco_single7/pose", PoseStamped, callback7)
    sub8 = rospy.Subscriber("/aruco_single8/pose", PoseStamped, callback8)
    sub9 = rospy.Subscriber("/aruco_single9/pose", PoseStamped, callback9)

    sub10 = rospy.Subscriber("/aruco_single10/pose", PoseStamped, callback10)
    sub11 = rospy.Subscriber("/aruco_single11/pose", PoseStamped, callback11)
    sub12 = rospy.Subscriber("/aruco_single12/pose", PoseStamped, callback12)
    sub13 = rospy.Subscriber("/aruco_single13/pose", PoseStamped, callback13)
    sub14 = rospy.Subscriber("/aruco_single14/pose", PoseStamped, callback14)

print ("######### Writer is Running##########")
with open('filex.csv','w') as csv_file:
    save_pos()
    print ("######### Writtingggg ############")
print ("Writing finished")