#! /usr/bin/env python

"""
.. module:: service
  :platform: Unix
  :synopsis: Python ROS 'service' node module for RT1 Second Assignment

.. moduleauthor:: Claudio Tomaiuolo c.tomaiuolo.rob@outlook.com

This node implements a service to collect the number of goal reached and deleted.
  
Service:
	service
    
Subscriber:
    /reaching_goal/result
"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from assignment_2_2022.srv import goals, goalsResponse



GoalsReached = 0 #Counts reached goals
GoalsCancelled = 0 #Counts cancelled goals


#Sends to the Subscriber the goals info
def Results(msg):
	"""
    Callback function that checks the status of the robot and increments the corresponding counters
    """
	Status=msg.status.status

	global GoalsCancelled, GoalsReached
	if Status==2: #Cancelled goal (status=2)
		GoalsCancelled+=1
	elif Status==3: #Reached goal (status=3)
		GoalsReached+=1


#Service function
def data(req):
	"""
    Function that returns the values of counters
    """
	global GoalsCancelled, GoalsReached
	return goalsResponse(GoalsReached, GoalsCancelled)


def main():
	"""
    This function inizializes the ROS node 'service'. It subscribes goal results and creates a service for giving to the user the number of
    goal reached and goal canceled.
    """
	rospy.init_node('service')
	
	srv=rospy.Service('service',goals,data)
	
	SubResults=rospy.Subscriber('/reaching_goal/result',assignment_2_2022.msg.PlanningActionResult,Results)

	rospy.spin()


if __name__=="__main__":
    main()
