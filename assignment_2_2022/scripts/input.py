#!/usr/bin/env python

"""
.. module:: input
  :platform: Unix
  :synopsis: Python ROS 'input' node module for RT1 Second Assignment

.. moduleauthor:: Claudio Tomaiuolo c.tomaiuolo.rob@outlook.com

This node takes as input from the keyboard the coordinates of the destination the robot is to reach.

Publisher:
  /pos_vel

Subscriber:
  /odom

ActionClient:
  /reaching_goal
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import position_velocity
import sys
import select

def PublishValues(msg):
  """
  Callback function to set position coordinates and velocity along x and y.

  Args:
  message(position_velocity)
  """
  global Pub

  Position = msg.pose.pose.position #Get the position
  Velocity = msg.twist.twist.linear	#Get the twist

  PosVel = position_velocity() #Create custom message

  PosVel.CurrentX=Position.x
  PosVel.CurrentY=Position.y
  PosVel.VelX=Velocity.x
  PosVel.VelY=Velocity.y

  Pub.publish(PosVel) #Publish the custom message


def ClientFunc():
  """
  Function that creates an action client, waiting for the server to be active,
  receives input from the user via the keyboard regarding the coordinates to be reached by the robot,
  creates a goal for the robot and sends it to the server.
  """
  Client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction) #Create the action client
  Client.wait_for_server()

  while not rospy.is_shutdown():
    #Get the coordinates from keyboard
    DestinationX=0
    DestinationY=0
    while True:
      try:
        DestinationX = float(input("Position X: "))
        DestinationY = float(input("Position Y: "))     
      except ValueError:
        print("Please, insert numbers.\n")
        continue
      else:
        break 
  
    #Create the goal for the robot
    goal = assignment_2_2022.msg.PlanningGoal()
    goal.target_pose.pose.position.x = DestinationX
    goal.target_pose.pose.position.y = DestinationY

    Client.send_goal(goal) #Send the goal to the server
    
    #The user has 10 seconds in order to cancel the goal by typing 'c'
    print("Enter 'c' to cancel the goal:")
    val = select.select([sys.stdin], [], [], 10)[0]
    if val:
      value = sys.stdin.readline().rstrip()
      if (value == "c"):
        print("Goal cancelled!")
        Client.cancel_goal()

def main():
  """
  This function initializes the ROS node 'input'. It publishes a "position_velocity" message and
  waits for the robot's position and velocity from topic '/odom'.
  """
  rospy.init_node('input')

  global Pub
  Pub=rospy.Publisher("/pos_vel",position_velocity,queue_size=1) #Send a message with velocity and position

  SubOdom=rospy.Subscriber("/odom",Odometry,PublishValues) #Get from "Odom" velocity and position
  
  ClientFunc()

if __name__=='__main__':
    main()