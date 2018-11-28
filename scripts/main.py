#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from road_processing_planning.srv import *
import time

from path_points_processor import *

def service_callback(place_name):

	path = path_processing_planning()
	return path.return_path(place_name)

if __name__ == '__main__':
	try:
		
		rospy.init_node('road_processor_planner', anonymous=True)

		path_getter_srv = rospy.Service('path_getter', getPath, service_callback)

		rospy.spin()

	except Exception as e:
		print(e)


#'Universidad Carlos III de Madrid, 30, Avenida de la Universidad'