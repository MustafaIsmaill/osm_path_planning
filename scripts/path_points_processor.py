#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point
import urllib
import yaml

import time
import math
import utm
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
		self.file_path= os.path.dirname(os.path.abspath(__file__))
		document = open(self.file_path[:len (self.file_path) -7] + 'config/document.yaml', 'r')
		parsed = yaml.load(document)
		self.place_name= parsed['place_name']
		self.grid_map_size= parsed['grid_map_size']

		# rospy.spin()


	
	def get_start(self):

		# rospy.Subscriber('/fix', NavSatFix, self.get_start)
		
		gps= rospy.wait_for_message('ada/fix', NavSatFix, timeout=None)
		self._start_lon= gps.longitude
		self._start_lat=gps.latitude
		self.latlon_start= (self._start_lat, self._start_lon)
		self._start_UTMx, self._start_UTMy, _, _ = utm.from_latlon(self._start_lat,self._start_lon )
	def get_end(self, end_points):
		
		self._endx=end_points.x
		self._endy=end_points.y
		
	def get_map(self, name):		
		
		
		


		file_path_map= self.file_path[:len (self.file_path) -7] + 'maps/'
		
		self.file_path_subgraph=self.file_path[:len (self.file_path) -7] + 'subgraphs/'

		try: 
			self._graph_proj= ox.load_graphml( name +'.xml', folder= file_path_map)
					
		except:

			try:

				rospy.loginfo("First Downloading Attempt")
				self._graph = ox.graph_from_place(name, network_type='drive')
				self._graph_proj = ox.project_graph(self._graph)
				ox.save_graphml(self._graph_proj ,folder= file_path_map , filename=name +'.xml')
			except:
				rospy.loginfo("First Downloading Attempt Failed, Retrying Download")
				self._graph = ox.graph_from_address(name, distance=250, network_type='drive')
				self._graph_proj = ox.project_graph(self._graph)
				ox.save_graphml(self._graph_proj ,folder= file_path_map ,  filename=name +'.xml')
		rospy.loginfo("Map Loaded Successfully")


		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)

	def plan_path(self):

		self._startx = self._start_UTMx
		self._starty = self._start_UTMy

		# self._endx = 434763
		# self._endy = 4464870

		# self._endx = 442028.32		//retiro park
		# self._endy = 4474037.16    //retiro park


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
		#ox.plot_graph_route(self._graph_proj, self._route,route_linewidth=6)

		for j in range(0, len(self._edges)):
			
			if ((self._edges.u[j] == self._route[0]) and 
				(self._edges.v[j] == self._route[1])):
				
				print("first edge")
				# print(self._edges.geometry[j])
				self._first_edge_geom = self._edges.geometry[j]
			elif((self._edges.u[j] == self._route[len(self._route)-2]) and 
				(self._edges.v[j] == self._route[len(self._route)-1])):
				
				print("last edge")
				# print(self._edges.geometry[j])
				self._last_edge_geom = self._edges.geometry[j]
		
	def generate_path_points(self):

		for r in range(0, len(self._route)-1):
			for i in range(0, len(self._edges)):
				if (self._edges.u[i] == self._route[r]) and (self._edges.v[i] == self._route[r+1]):
					for g in range(0, len(self._edges.geometry[i].xy[0])):
						self._route_pointx.append(self._edges.geometry[i].xy[0][g])
						self._route_pointy.append(self._edges.geometry[i].xy[1][g])
		

		self._projected_start_point= self._first_edge_geom.interpolate(
			self._first_edge_geom.project(Point(self._startx, self._starty)))

		print(self._projected_start_point)

		self._projected_end_point=self._last_edge_geom.interpolate(
			self._last_edge_geom.project(Point(self._endx, self._endy)))

		print(self._projected_end_point)

		for i in range (0, len(self._route_pointx)):

			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=i

			if i == 0:
				pose_st.header.stamp=rospy.Time.now()
			 	pose_st.pose.position.x = self._projected_start_point.x
			 	pose_st.pose.position.y = self._projected_start_point.y
			elif i == len(self._route_pointx):
				pose_st.header.stamp=rospy.Time.now()
				pose_st.pose.position.x = self._projected_end_point.x
				pose_st.pose.position.y = self._projected_end_point.y
			else:
				pose_st.header.stamp=rospy.Time.now()
				pose_st.pose.position.x = self._route_pointx[i]
				pose_st.pose.position.y = self._route_pointy[i]

		 	self._path.poses.append(pose_st)

		self.plot_route_points()
		self._old_UTMx=self._start_UTMx
		self._old_UTMy=self._start_UTMy
		self.curr_gps = rospy.Subscriber('ada/fix', NavSatFix,self.draw_subgraph)


	def draw_subgraph(self,curr_gps):

		
		
		self.curr=curr_gps

		rospy.loginfo("******************")


		self._curr_lat=self.curr.latitude
		self._curr_lon=self.curr.longitude
		if(math.isnan(self.curr.latitude)==False):
			
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
				self.curr_gps_point=(self._curr_lat,self._curr_lon)
				north, south, east, west= ox.bbox_from_point(self.curr_gps_point, distance=self.grid_map_size)
				rospy.loginfo(north)
				rospy.loginfo(south)
				rospy.loginfo(east)
				rospy.loginfo(west)
				url_name = 'https://overpass-api.de/api/map?bbox=' + str(west) + "," + str (south) + "," + str(east) + "," + str(north)
				print(url_name)

				urllib.urlretrieve(url_name, self.file_path_subgraph + 'subgraph.xml')

			
			else:
				rospy.loginfo("distance is less than 25 meters")
		else:
			rospy.loginfo("nan signal")

	def plot_route_points(self):
		fig, ax = plt.subplots()
		self._edges.plot(ax=ax)
		self._nodes.plot(ax=ax)
		
		ax.plot(self._startx,self._starty, 'r+')
		ax.plot(self._endx,self._endy, 'r+')


		for m in range(0,len(self._path.poses)):
			# plt.plot(self._path.poses[m].pose.position.x,self._path.poses[m].pose.position.y)
			
			ax.plot(self._path.poses[m].pose.position.x,self._path.poses[m].pose.position.y ,'ro')


		ax.plot(self._path.poses[0].pose.position.x,self._path.poses[0].pose.position.y,'b+')
		ax.plot(self._path.poses[len(self._path.poses)-1].pose.position.x,
			self._path.poses[len(self._path.poses)-1].pose.position.y,'b+')

		#plt.show()
		
	def return_path(self, goal):
		
		self.get_map(self.place_name)
		self.get_start()
		self.get_end(goal)
		self.plan_path()
		self.generate_path_points()
		#self.plot_route_points()
		rospy.loginfo("************Done**********")
		return self._path