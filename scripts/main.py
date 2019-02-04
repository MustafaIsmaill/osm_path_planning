#!/usr/bin/env python

import rospy
from map_processor import *
from Path import *
from local_map import *

def gps_callback(gps_msg):
	"""
		This is subscriber's callback that publishes the path points in ROS
		and generates the local_map.

		parameters:
		------
		gps_msg : sensor_msgs.msg._NavSatFix.NavSatFix
			this contains the lon and lat positions for the car from GPS


	"""
	osm_path.publish_path()
	sub_graph.draw_local_map(gps_msg)

if __name__ == '__main__':
	try:

		"""
		initializes ROS node 'road_processor_planner', then processes the OSM
		map, generates the path and then enters the subscriber's callback to
		get the vehicle's position and generate local_maps accordingly.

		Raises:
		--------- 
		Exception

			Depending on where the error occured.			

		"""
		rospy.init_node('road_processor_planner', anonymous=True)

		osm_map = map_processing()		
		current_map = osm_map.get_map(rospy.get_param("place_name"))
		edges = osm_map.get_edges()

		osm_path = path_generator(current_map, edges)
		start_x, start_y = osm_path.get_start_and_goal()
		# osm_path.plan_path()
		# osm_path.generate_path_points()
		# osm_path.shift_path_points()
		# osm_path.plot_route_points()

		osm_path.make_fake_path()

		sub_graph = local_map(start_x, start_y)

		sub = rospy.Subscriber('ada/fix', NavSatFix, gps_callback)

		rospy.spin()

	except Exception as e:
		print(e)