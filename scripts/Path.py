#!/usr/bin/env python


import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point

import osmnx as ox
import networkx as nx
import utm
import math
import time

from collections import OrderedDict	
import matplotlib.pyplot as plt
from shapely.geometry import Point

import pandas as pd
import numpy as np

from heapq import heappush, heappop
from itertools import count

import shift_path as sp


class path_generator:
	"""
	The path_generator class subscribes '/move_base_simple/goal' topic to get start 
	point and then subscribes to 'ada/fix' to get goal point then provides the path
	between them.
	
	...

	Parameters:
	---------
	current_map : networkx MultiDiGraph
		the downloaded networkx graph containing the road network

	edges: geopandas GeoDataFrame	
		the edges contain the co-ordinates of roads in map
		
	Methods:
	------
	get_start_and_goal()
		susubscribes '/move_base_simple/goal' topic to get start 
		point and then subscribes to 'ada/fix' to get goal point
	
	plan_path()
		finds the nearests nodes to the start and end co-ordinates recieved
		and then uses Dijkstra's Algorithm to find shortest distance between 
		these nodes
	generate_path_points()
		sorts the path points and appends them in a logical way depending
		on a set of cases
	get_nearest_edge(Graph, edge)
		projects the GPS point to the nearest road
	get_nearest_node(Graph, Point)
		finds the nearest node to the GPS point to start route planning
	publish_path()
		publishes PoseStamped ROS msg 
	plot_route()
		plots a map containing the route points and streets

	calc_distance()
		calculates the euclidean distance between two graph co-ordinates
	projected_point_before_start_node()
		checks if the projected start point lies before of after the route 
		start node
	projected_point_before_goal_node()
		checks if the projected goal point lies before of after the route 
		goal node
	"""
	def __init__(self,current_map, edges):
	

		"""
		parameters

		-------------
		current_map : networkx MultiDiGraph
			the downloaded networkx graph containing the road network

		edges: geopandas GeoDataFrame	
			the edges contain the co-ordinates of roads in map
		"""

		rospy.loginfo("Preparing Path")
		self._route_pointx = []
		self._route_pointy = []
		self._graph_proj= current_map
		self._edges= edges

		self.map_frame=rospy.get_param("map_frame")
		self._NavSatFix= NavSatFix()

		self._path = Path()
		self._path.header.stamp = rospy.Time.now()
		self._path.header.frame_id = self.map_frame

		self.route_pub = rospy.Publisher("route_points", Path, queue_size=1)
		rate = rospy.Rate(1)

	def get_start_and_goal(self):

		"""

		Returns Start and Goal points in UTM co-ordinates. 
		First subscribes to /move_base_simple/goal' and waits till UTM goal points are passed
		in a PoseStamped ROS msg and then store them as a tuple. Then waits for the first GPS 
		Reading from 'ada/fix' then converts it from lat lon system to UTM system and stores
		this point as tuple.

		Returns
		-------------
		self._startx : float64
		self._starty : float64
		"""
		rospy.loginfo("waiting for goal point")
		self.goal_points= rospy.wait_for_message('/move_base_simple/goal', PoseStamped, timeout=None)
		self._goalx= self.goal_points.pose.position.x
		self._goaly= self.goal_points.pose.position.y
		self._goal_point=(self._goaly,self._goalx)



		rospy.loginfo("waiting for start point")
		gps = rospy.wait_for_message('ada/fix', NavSatFix, timeout=None)

		self._start_lon, self._start_lat = gps.longitude, gps.latitude
		self._startx, self._starty, _, _ = utm.from_latlon(self._start_lat, self._start_lon )
		self._start_point=(self._starty, self._startx)
		return self._startx,self._starty

	def plan_path(self):
		
		"""
		it generates OSM nodes for the shortest path using dijsktra's algorithm 
		between two nodes. This happens by first projecting the start and end points 
		to their corresponding nearest edges. The start and end node in the first edge 
		are extracted, and if this edge (road in real life) is has oneway tag from OSM,
		Then the first node of the path must to be the end node of edge, else, the 	
		first node is free to be selected according the distance. The goal node  is
		then set using get_nearest_node method. The projected map, start and goal 
		nodes are passed to the dijsktra_path method that returns a list of 
		the nodes that form the path.

		"""


		self.first_edge_geometry, self.first_u, self.first_v = self.get_nearest_edge(self._graph_proj, self._start_point)
		self.last_edge_geometry, self.goal_v , self.goal_u = self.get_nearest_edge(self._graph_proj, self._goal_point)
		
		self._projected_start_point= self.first_edge_geometry.interpolate(
			self.first_edge_geometry.project(Point(self._startx, self._starty)))


		self._projected_goal_point=self.last_edge_geometry.interpolate(
			self.last_edge_geometry.project(Point(self._goalx, self._goaly)))
		for j in range(0, len(self._edges)):
			if ((self._edges.v[j] == self.first_v) and 
				(self._edges.u[j] == self.first_u)):


				self.oneway_edge= self._edges.oneway[j]
				self.start_node=self._edges.v[j]


		if (self.oneway_edge == True):
			rospy.loginfo("one way")
			self._origin_node=	self.start_node


		else:

			self._origin_node = self.get_nearest_node(self._graph_proj, self._start_point)
		
		self._DiGraph= nx.DiGraph()		
		
		for j in range(0, len(self._edges)):
			self._DiGraph.add_edge(self._edges.u[j],self._edges.v[j])

		self._goal_node = self.get_nearest_node(self._graph_proj, self._goal_point)
		self._route = nx.dijkstra_path(G= self._graph_proj, source= self._origin_node,
		 target=self._goal_node , weight='length')
		# self._route = self.astar_path(G= self._DiGraph, source= self._origin_node,
		#  target=self._goal_node , weight='length', heuristic= None)



	def generate_path_points(self):

		"""
		This is used to transform path from osm Nodes to path points in UTM system.It
		also appends projected start and goal points in a the correct sequence. It 
		starts by find UTM coordinates for each node in path and append them to list
		and removing duplicated path points..
		Then starts appending the start and goal projected points depending one
		different cases.
		All of the route points are appended to pose_st object to be published later.

		First check if the first edge is one way, then append projected start point 
		first then append the rest of the path except the last node.
		Else if the road is not one way, If the projected start point is before goal,
		then this point is appended. Else the first path node is replaced by 
		the projected first point.Secondly the rest of the path points are appended 
		to pose_st. Third If projected  goal point is before last path node, 
		then projected goal point is appended to list path. Else, the last node is appended 
		followed by the projected point. Else if the road is not one way, If the 
		projected start point is before goal,then this point is appended. Else the 
		first path node is replaced by the projected first point.

		"""

		for r in range(0, len(self._route)-1):
			
			for i in range(0, len(self._edges)-1):
			
				if (self._edges.u[i] == self._route[r]) and (self._edges.v[i] == self._route[r+1]):
			
					for g in range(0, len(self._edges.geometry[i].xy[0])):
						self._route_pointx.append(self._edges.geometry[i].xy[0][g])
						self._route_pointy.append(self._edges.geometry[i].xy[1][g])
		
		self._route_pointx = list(OrderedDict.fromkeys(self._route_pointx))
		self._route_pointy = list(OrderedDict.fromkeys(self._route_pointy))

		if (self.oneway_edge == True):
			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=0
			pose_st.header.frame_id = self.map_frame
			pose_st.pose.position.x = self._projected_start_point.x
			pose_st.pose.position.y = self._projected_start_point.y
			self._path.poses.append(pose_st)
			s=0
			seq=1

		elif ( self.projected_point_before_start_node()):

			pose_st = PoseStamped()
			
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=0
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x = self._projected_start_point.x
			pose_st.pose.position.y = self._projected_start_point.y
			self._path.poses.append(pose_st)

			pose_st = PoseStamped()

			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=1
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x=self._route_pointx[0]
			pose_st.pose.position.y=self._route_pointy[0]
			self._path.poses.append(pose_st)
			s=1
			seq=2
		
		else:

			self._route_pointx[0]=self._projected_start_point.x
			self._route_pointy[0]=self._projected_start_point.y
			s=0
			seq=1

		for i in range (s, len(self._route_pointx)-1):


			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=seq
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x = self._route_pointx[i]
			pose_st.pose.position.y = self._route_pointy[i]
			self._path.poses.append(pose_st)
		 	seq+=1

		if (self.projected_point_before_goal_node()):

			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=seq
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x=self._projected_goal_point.x
			pose_st.pose.position.y=self._projected_goal_point.y
			self._path.poses.append(pose_st)

		else:
			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=seq
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x=self._route_pointx[(len(self._route_pointx)-1)]
			pose_st.pose.position.y=self._route_pointy[(len(self._route_pointy)-1)]
			self._path.poses.append(pose_st)
			
			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=seq+1
			pose_st.header.frame_id=self.map_frame
			pose_st.pose.position.x=self._projected_goal_point.x
			pose_st.pose.position.y=self._projected_goal_point.y
			self._path.poses.append(pose_st)
	
		# x = []
		# y = []
		# for p in self._path.poses:
		# 	x.append(p.pose.position.x)
		# 	y.append(p.pose.position.y)
			
		# rospy.loginfo(x)
		# rospy.loginfo(y)

		self.old_UTMx=self._startx
		self.old_UTMy=self._starty

	def publish_path(self):
		"""
		publishes nav_msgs.msg._Path.Path ROS msg 

		"""
		self.route_pub.publish(self._path)

	def plot_route_points(self):
		fig, ax = plt.subplots()
		self._edges.plot(ax=ax)

		ax.plot(self._startx,self._starty, 'r+')
		ax.plot(self._goalx,self._goaly, 'r+')


		for m in range(0,len(self._path.poses)):
			plt.plot(self._path.poses[m].pose.position.x,self._path.poses[m].pose.position.y)
			
			ax.plot(self._path.poses[m].pose.position.x,self._path.poses[m].pose.position.y ,'ro')


		ax.plot(self._path.poses[0].pose.position.x,self._path.poses[0].pose.position.y,'b+')
		ax.plot(self._path.poses[len(self._path.poses)-1].pose.position.x,
			self._path.poses[len(self._path.poses)-1].pose.position.y,'b+')

		plt.show()

	def get_nearest_edge(self,G, point):
	    """
	    Return the nearest edge to a pair of coordinates. Pass in a graph and a tuple
	    with the coordinates. We first get all the edges in the graph. Secondly we compute
	    the euclidean distance from the coordinates to the segments determined by each edge.
	    The last step is to sort the edge segments in ascgoaling order based on the distance
	    from the coordinates to the edge. In the goal, the first element in the list of edges
	    will be the closest edge that we will return as a tuple containing the shapely
	    geometry and the u, v nodes.

	    Parameters
	    ----------
	    G : networkx multidigraph
	    point : tuple
	        The (lat, lng) or (y, x) point for which we will find the nearest edge
	        in the graph

	    Returns
	    -------
	    closest_edge_to_point : tuple (shapely.geometry, u, v)
	        A geometry object representing the segment and the coordinates of the two
	        nodes that determine the edge section, u and v, the OSM ids of the nodes.
	    """
	   
	    graph_edges = self._edges[["geometry", "u", "v"]].values.tolist()

	    edges_with_distances = [
	        (
	            graph_edge,
	            Point(tuple(reversed(point))).distance(graph_edge[0])
	        )
	        for graph_edge in graph_edges
	    ]

	    edges_with_distances = sorted(edges_with_distances, key=lambda x: x[1])
	    closest_edge_to_point = edges_with_distances[0][0]

	    geometry, u, v = closest_edge_to_point

	    return geometry, u, v
	def calc_distance(self, point1, point2):
		"""
			calculates euclidean distance between two given points

		"""
		x1, y1 = point1[0], point1[1]
		x2, y2 = point2[0], point2[1]

		return math.sqrt((x1-x2)**2 + (y1-y2)**2)
	def projected_point_before_start_node(self):

		"""
			checks if the projected point is before the first node or not.
			this done by comparing euclidean distance between second node in
			path and projected point to the edge length.


			Returns
			----------------
				True
					if distance between second node in
					path and projected point is less then the edge length

		"""
		route_point1=(self._route_pointx[0],self._route_pointy[0])
		route_point2=(self._route_pointx[1],self._route_pointy[1])
		candidate_point=(self._projected_start_point.x,self._projected_start_point.y)

		if (self.calc_distance(route_point2,candidate_point) > self.calc_distance(route_point2,route_point1)):

			return True
	
	def projected_point_before_goal_node(self):

		"""
			checks if the projected point is before the last node or not.
			this done by comparing euclidean distance between last node in
			path and projected point to the edge length.


			Returns
			----------------
				True
					if the distance between last node in
					path and projected point is less then the edge length

		"""

		route_point1=(self._route_pointx[len(self._route_pointx)-2],self._route_pointy[len(self._route_pointx)-2])
		route_point2=(self._route_pointx[len(self._route_pointx)-1],self._route_pointy[len(self._route_pointx)-1])
		candidate_point=(self._projected_goal_point.x,self._projected_goal_point.y)	

		if (self.calc_distance(route_point1,route_point2) > self.calc_distance(route_point1,candidate_point)):

			return True
	def get_nearest_node(self,G, point):

		"""
	    Return the graph node nearest to some specified (lat, lng) or (y, x) point,
	    and optionally the distance between the node and the point. This function
		uses euclidean distance calculator.

	    Parameters
	    ----------
	    G : networkx multidigraph
	   
	    point : tuple
	        The (lat, lng) or (y, x) point for which we will find the nearest node
	        in the graph
		"""
		coords = np.array([[node, data['x'], data['y']] for node, data in G.nodes(data=True)])
		df = pd.DataFrame(coords, columns=['node', 'x', 'y']).set_index('node')
		df['reference_y'] = point[0]
		df['reference_x'] = point[1]
		distances = self.euclidean_dist_vec(y1=df['reference_y'],x1=df['reference_x'],y2=df['y'],x2=df['x'])
		nearest_node = int(distances.idxmin())
		return nearest_node

	def euclidean_dist_vec(self,y1, x1, y2, x2):
		distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
		return distance
		
	def astar_path(self,G, source, target,  weight, heuristic):
		"""
		Return a list of nodes in a shortest path between source and target
		using the A* ("A-star") algorithm.

		There may be more than one shortest path.  This returns only one.

		Parameters
		----------
		G : NetworkX graph

		source : node
		Starting node for path

		target : node
		Ending node for path

		heuristic : function, optional
		A function to evaluate the estimate of the distance
		from the a node to the target.  The function takes
		two nodes arguments and must return a number.

		weight: string, optional
		Edge data key corresponding to the edge weight.

		Raises
		------
		Value Error
		If no path exists between source and target
		"""
		if source not in G or target not in G:
			msg = 'Either start node or target node is not in map'
			raise ValueError(msg)

		if heuristic is None:
        
			def heuristic(u, v):
				return 0

		push = heappush
		pop = heappop

		c = count()
		queue = [(0, next(c), source, 0, None)]

		enqueued = {}
		explored = {}

		while queue:
			
			_, __, curnode, dist, parent = pop(queue)
			
			if curnode == target:
				path = [curnode]
				node = parent
				while node is not None:
					path.append(node)
					node = explored[node]
				path.reverse()
				return path

			if curnode in explored:
				continue

			explored[curnode] = parent

			for neighbor, w in G[curnode].items():
				
				if neighbor in explored:
					continue
				fcost = dist + w.get(weight, 1)
				
				if neighbor in enqueued:
					qcost, h = enqueued[neighbor]

					if qcost <= fcost:
						continue
				else:
					h = heuristic(neighbor, target)
				enqueued[neighbor] = fcost, h
				push(queue, (fcost + h, next(c), neighbor, fcost, curnode))
				
		msg = 'Can not find path from start to end nodes'
		raise ValueError(msg)

	def shift_path_points(self):
		x = []
		y = []
		for p in self._path.poses:
			x.append(p.pose.position.x)
			y.append(p.pose.position.y)

		x, y = sp.rm_duplicates(x, y)
		lines = sp.shift_path(x, y, float(rospy.get_param("lane_shift")))
		lines = sp.remove_intersects(lines)

		new_path = Path()
		new_x = []
		new_y = []
		for line in lines:
			xl, yl = line.xy
			for idx in range(len(xl)):
				new_x.append(xl[idx])
				new_y.append(yl[idx])

		new_x, new_y = sp.rm_duplicates(new_x, new_y)
	
		for idx in range(len(new_x)):
			pose_st = PoseStamped()
			pose_st.header.stamp = rospy.Time.now()
			pose_st.header.frame_id = self.map_frame
			pose_st.header.seq = idx
			pose_st.pose.position.x = new_x[idx]
			pose_st.pose.position.y = new_y[idx]
			new_path.poses.append(pose_st)

		self._path = new_path
