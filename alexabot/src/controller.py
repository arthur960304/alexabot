#!/usr/bin/env python2.7
import rospy
import roslib
import actionlib
import sys
import numpy as np
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from alexabot.srv import Command
from alexabot.srv import CommandRequest
from alexabot.srv import CommandResponse

#Add Map points
p1x = 0.0
p1y = 0.0

p2x = 7.827
p2y = -0.729

p3x = 12.614
p3y = -6.445


def moveToGoal(xGoal, yGoal):
    rospy.loginfo("Ready to move ...")

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    
    goal = MoveBaseGoal()
    
    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # set goal point
    goal.target_pose.pose.position =  Point(xGoal, yGoal, 0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = -1
    goal.target_pose.pose.orientation.w = 1.0
    
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    # moving towards the goal
    ac.wait_for_result()
    
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False

def handle_command(req):
    loc = req.location
    print ("Navigate called:", loc)

    if(loc=="1"):
        moveToGoal(p1x, p1y)
        return CommandResponse("Success")

    elif(loc=="2"):
        moveToGoal(p2x, p2y)
        return CommandResponse("Success")

    elif(loc=="3"):
        moveToGoal(p3x, p3y)
        return CommandResponse("Success")

    else:
        print("Others location called:", loc)
        return CommandResponse("False")



def main(args):
    rospy.init_node('controller', anonymous=True)
    command_srv = rospy.Service("voice_commands", Command, handle_command)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        rospy.shutdown();

if __name__ == '__main__':
    main(sys.argv)
