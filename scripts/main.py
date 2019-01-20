#!/usr/bin/env python

import rospy
from map_processor import *
from Path import *
from Subgraph import *

def gps_callback(gps_msg):
	sub_graph.draw_subgraph(gps_msg)
	osm_path.publish_path()

if __name__ == '__main__':
	try:

		rospy.init_node('road_processor_planner', anonymous=True)

		osm_map = map_processing()		
		current_map = osm_map.get_map(rospy.get_param("place_name"))
		edges = osm_map.get_edges()
		
		osm_path = path_generator(current_map, edges)
		start_x, start_y = osm_path.get_start_and_end()
		rospy.loginfo("**")
		osm_path.plan_path()
		osm_path.generate_path_points()
		osm_path.plot_route_points()
		
		sub_graph = subgraph(start_x, start_y)

		sub = rospy.Subscriber('ada/fix', NavSatFix, gps_callback)

		rospy.spin()

	except Exception as e:
		print(e)