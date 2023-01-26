#! /usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from assignment_2_2022.srv import goals, goalsResponse
# from assignment_2_2022.srv import goal_srv goal_srvResponse


GoalsReached = 0 #Counts reached goals
GoalsCancelled = 0 #Counts cancelled goals


#Sends to the Subscriber the goals info
def Results(msg):
	Status=msg.status.status

	global GoalsCancelled, GoalsReached
	if Status==2: #Cancelled goal (status=2)
		GoalsCancelled+=1
	elif Status==3: #Reached goal (status=3)
		GoalsReached+=1


#Service function
def data(req):
	global GoalsCancelled, GoalsReached
	return goalsResponse(GoalsReached, GoalsCancelled)


def main():
	rospy.init_node('service')
	
	srv=rospy.Service('service',goals,data)
	
	SubResults=rospy.Subscriber('/reaching_goal/result',assignment_2_2022.msg.PlanningActionResult,Results)

	rospy.spin()


if __name__=="__main__":
    main()