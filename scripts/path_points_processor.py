#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import time
import cPickle as pickle

class path_processing_planning:

	def __init__(self):
		
		# rospy.init_node('road_processor_planner', anonymous=True)

		# self.route_pub = rospy.Publisher("route_points", Path)
		# self._rate = rospy.Rate(1)

		self._route_pointx = []
		self._route_pointy = []

		self._path = Path()
		self._path.header.stamp = rospy.Time.now()

		# rospy.spin()


	def get_map(self, name):		
		try: 
			with open("/home/ahmad/catkin_ws/src/road_processing_planning/maps/" + name +'.p', 'rb') as f:
				self._graph = pickle.load(f) 
		except:
			
			try:

				self._graph = ox.graph_from_place(name, network_type='drive')
			except:
				self._graph = ox.graph_from_address(name, distance=250, network_type='drive')
	
			with open("/home/ahmad/catkin_ws/src/road_processing_planning/maps/"+ name+ '.p', 'wb') as f:
				pickle.dump(self._graph, f)

		self._graph_proj = ox.project_graph(self._graph)
		
		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)
	
	def plan_path(self):

		self._startx = self._nodes[:].x[(self._edges[:].v[34])]
		self._starty = self._nodes[:].y[(self._edges[:].v[34])]
		self._endx = self._nodes[:].x[(self._edges[:].u[34])]
		self._endy = self._nodes[:].y[(self._edges[:].u[34])]

		self._origin=(self._startx,self._starty)
		self._destination=(self._endx,self._endy)
		self._origin_node = ox.get_nearest_node(self._graph_proj, self._origin)
		self._destination_node = ox.get_nearest_node(self._graph_proj, self._destination)

		self._route = nx.dijkstra_path(G= self._graph_proj, source= self._origin_node,
		 target=self._destination_node , weight='length')
		
	def generate_path_points(self):

		for r in range(0, len(self._route)-1):
			for i in range(0, len(self._edges)):
				if (self._edges.u[i] == self._route[r]) and (self._edges.v[i] == self._route[r+1]):
					for g in range(0, len(self._edges.geometry[i].xy[0])):
						self._route_pointx.append(self._edges.geometry[i].xy[0][g])
						self._route_pointy.append(self._edges.geometry[i].xy[1][g])
		
		for i in range (0, len(self._route_pointx)):
			
			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=i

		 	pose_st.pose.position.x = self._route_pointx[i]
		 	pose_st.pose.position.y = self._route_pointy[i]

		 	self._path.poses.append(pose_st)

	# def publish_path_points(self):

	# 	while not rospy.is_shutdown():
	# 		print("publishing...")
	# 		self.route_pub.publish(self._path)
	# 		self._rate.sleep()

	def draw_route(self):
		ox.plot_graph_route(self._graph_proj, self._route,route_linewidth=6)

	def return_path(self, place_name):
		place_name = str(place_name)
		length = len(place_name)
		place_name = place_name[13:length-1]

		self.get_map(place_name)
		self.plan_path()
		self.generate_path_points()
		self.draw_route()

		return self._path
