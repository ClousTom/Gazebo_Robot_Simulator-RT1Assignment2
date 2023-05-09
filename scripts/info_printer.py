#! /usr/bin/env python

"""
.. module:: info_printer
  :platform: Unix
  :synopsis: Python ROS 'info_printer' node module for RT1 Second Assignment

.. moduleauthor:: Claudio Tomaiuolo c.tomaiuolo.rob@outlook.com

This node calculates the distance from the desired destination and the current position of the robot and its average speed.

Subscriber:
	/pos_vel
"""

import rospy
import math
import time
from scipy.spatial import distance #math.dist gets error in some computers
from assignment_2_2022.msg import position_velocity

InfoFreq = 1.0 #Frequency for printing infos
InfoPrinted = 0 #The last time that infos were printed


#Callback function
def PosVel(msg):
	"""
	Callback function of the Subscriber that converts time in milliseconds.
	It calculates the distance from the robot position and the desired destination and the average speed of the robot, printing them on the screen.

	Args:
	message(position_velocity)
	"""
	global InfoFreq, InfoPrinted
	Period=(1.0/InfoFreq)*1000 #Time in milliseconds
	CurrentTime=time.time()*1000

	if (CurrentTime-InfoPrinted)>Period:

		CurrentX=msg.CurrentX #Current position
		CurrentY=msg.CurrentY
		DestinationX=rospy.get_param("des_pos_x") #Position by user
		DestinationY=rospy.get_param("des_pos_y")
		
		CurrentPosition=(CurrentX,CurrentY)
		DestinationPosition=(DestinationX,DestinationY)
		Distance=distance.euclidean(DestinationPosition,CurrentPosition) #Euclidean distance

		Speed=math.sqrt(msg.VelX**2+msg.VelY**2) #Average speed

		print("Average speed: ",round(Speed,6)," Distance from selected position: ",round(Distance,6))

		InfoPrinted=CurrentTime
	

def main():
	"""
	This function inizializes the ROS node 'info_printer'. It gets the value of the frequency and
	waits the custon message from the topic '/pos_vel'.
	"""
	rospy.init_node('info_printer')
	
	global InfoFreq
	InfoFreq=rospy.get_param("freq") #Get the publish frequency
	
	SubPosVel=rospy.Subscriber("/pos_vel", position_velocity,PosVel) #Get from "pos_vel" a parameter
	
	rospy.spin()


if __name__=="__main__":
	main()	
