#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from road_processing_planning.srv import *
import time

from path_points_processor import *




if __name__ == '__main__':
	try:
		
		rospy.init_node('road_processor_planner', anonymous=True)
		path = path_processing_planning()
		path.return_path()

		# path.get_map(rospy.get_param("place_name"))

		rospy.spin()


	except Exception as e:
		print(e)
