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
		self._start_lon= gps.longitude
		self._start_lat=gps.latitude
		self.latlon_start= (self._start_lat, self._start_lon)
		self._start_UTMx, self._start_UTMy, _, _ = utm.from_latlon(self._start_lat,self._start_lon )
	def get_end(self, end_points):
		
		self._endx=end_points.x
		self._endy=end_points.y
		
	def get_map(self, name):		
		
		
		
		file_path= os.path.dirname(os.path.abspath(__file__))

		file_path_map= file_path[:len (file_path) -7] + 'maps/'
		
		self.file_path_subgraph=file_path[:len (file_path) -7] + 'subgraphs/'

		try: 
			with open(file_path_map + name +'.p', 'rb') as f:
				
				rospy.loginfo("Loading Offline map")
				self._graph = pickle.load(f) 
					
		except:
			
			try:

				rospy.loginfo("First Downloading Attempt")
				self._graph = ox.graph_from_place(name, network_type='drive')
				
			except:
				rospy.loginfo("First Downloading Attempt Failed, Retrying Download")
				self._graph = ox.graph_from_address(name, distance=250, network_type='drive')
				
			with open(file_path_map + name+ '.p', 'wb') as f:
				pickle.dump(self._graph, f)

		
		self._graph_proj = ox.project_graph(self._graph)

		x=time.time()
		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)
		y=time.time()
		print("time is :" + str (y-x))
	def plan_path(self):

		self._startx = self._start_UTMx
		self._starty = self._start_UTMy

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
		self._old_UTMx=self._start_UTMx
		self._old_UTMy=self._start_UTMy
		ox.plot_graph_route(self._graph_proj,self._route,route_linewidth=6)
		self.curr_gps = rospy.Subscriber('/fix', NavSatFix,self.draw_subgraph)
	def draw_subgraph(self,curr_gps):

		# ox.plot_graph_route(self._graph_proj, self._route,route_linewidth=6)
		
		self.curr=curr_gps

		rospy.loginfo("******************")


		self._curr_lat=self.curr.latitude
		self._curr_lon=self.curr.longitude
		
			
		self._curr_UTMx, self._curr_UTMy, _, _ = utm.from_latlon(self._curr_lat, self._curr_lon)

	

		xdiff =self._curr_UTMx - self._old_UTMx
		ydiff=self._curr_UTMy - self._old_UTMy
		
		rospy.loginfo("Distance in x directon: " + str (abs(xdiff)))
		rospy.loginfo("Distance in y directon: " + str (abs(ydiff)))
		if (abs(xdiff) > 25 or abs(ydiff) > 25):
				
			rospy.loginfo("Generating subgraph")
			self._curr_point=(self._curr_UTMy, self._curr_UTMx)
			self._curr_node = ox.get_nearest_node(self._graph_proj, self._curr_point, method='euclidean')
			self._old_UTMx=self._curr_UTMx
			self._old_UTMy=self._curr_UTMy
			subgraph1= nx.ego_graph(self._graph_proj,self._curr_node, radius=2, center=True, undirected=False, distance=None)
			fig, ax =ox.plot_graph(subgraph1)
			ox.save_graphml(subgraph1 , filename=self.file_path_subgraph + 'subgraph.xml')
			
		else:
			rospy.loginfo("distance is less than 25 meters")
	def return_path(self, goal):
		
		place_name= 'leganes,Spain'
		self.get_map(place_name)
		rospy.loginfo("Map Loaded Successfully")
		self.get_start()
		self.get_end(goal)
		self.plan_path()
		self.generate_path_points()


		return self._path
