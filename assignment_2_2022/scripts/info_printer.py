#! /usr/bin/env python

import rospy
import math
import time
from scipy.spatial import distance #math.dist gets error in some computers
from assignment_2_2022.msg import position_velocity


InfoFreq = 1.0 #Frequency for printing infos
InfoPrinted = 0 #The last time that infos were printed


#Callback function
def PosVel(msg):
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
	rospy.init_node('info_printer')
	
	global InfoFreq
	InfoFreq=rospy.get_param("freq") #Get the publish frequency
	
	SubPosVel=rospy.Subscriber("/pos_vel", position_velocity,PosVel) #Get from "pos_vel" a parameter
	
	rospy.spin()


if __name__=="__main__":
	main()	
