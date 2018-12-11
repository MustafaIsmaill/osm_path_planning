#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

import time
import utm
import cPickle as pickle
import os

class path_processing_planning:

	def __init__(self):
		
		# rospy.init_node('road_processor_planner', anonymous=True)

		# self.route_pub = rospy.Publisher("route_points", Path)
		# self._rate = rospy.Rate(1)
		
		self._route_pointx = []
		self._route_pointy = []

		self._NavSatFix= NavSatFix()

		self._path = Path()
		self._path.header.stamp = rospy.Time.now()

		# rospy.spin()


	
	def get_start(self):

		# rospy.Subscriber('/fix', NavSatFix, self.get_start)
		gps= rospy.wait_for_message('/fix', NavSatFix, timeout=None)
		self._longitude= gps.longitude
		self._latitude=gps.latitude
		self._UTMx, self._UTMy, _, _ = utm.from_latlon(self._latitude, self._longitude)
	def get_end(self, end_points):
		
		self._endx=end_points.x
		self._endy=end_points.y
		
	def get_map(self, name):		
		
		file_path= os.path.dirname(os.path.abspath(__file__))

		file_path= file_path[:len (file_path) -7] + 'maps/'

		try: 
			with open(file_path + name +'.p', 'rb') as f:
				self._graph = pickle.load(f) 
		except:
			
			try:

				self._graph = ox.graph_from_place(name, network_type='drive')
			except:
				self._graph = ox.graph_from_address(name, distance=250, network_type='drive')
	
			with open(file_path + name+ '.p', 'wb') as f:
				pickle.dump(self._graph, f)

		self._graph_proj = ox.project_graph(self._graph)
		
		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)
	
	def plan_path(self):

		self._startx = self._UTMx
		self._starty = self._UTMy

		# self._endx = 434763
		# self._endy = 4464870


		self._origin=(self._starty,self._startx)
		rospy.loginfo("start point from ROSbag: (UTMy,UTMx)")
		rospy.loginfo(self._origin) 
		self._destination=(self._endy,self._endx)
		rospy.loginfo("Goal Point Entered: (UTMy,UTMx)")
		rospy.loginfo(self._destination) # (434764 4464870)
		self._origin_node = ox.get_nearest_node(self._graph_proj, self._origin, method='euclidean')
		self._destination_node = ox.get_nearest_node(self._graph_proj, self._destination, method= 'euclidean')
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

	def draw_route(self):
		ox.plot_graph_route(self._graph_proj, self._route,route_linewidth=6)
		self._subgraph= self._graph_proj.subgraph(self._route)
		fig, ax = ox.plot_graph(self._subgraph)

	def return_path(self, goal):
		
		place_name= 'leganes,Spain'
		self.get_end(goal)
		self.get_start()
		self.get_map(place_name)
		self.plan_path()
		self.generate_path_points()
		self.draw_route()

		return self._path
