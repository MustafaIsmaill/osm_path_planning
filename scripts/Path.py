#!/usr/bin/env python


import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point

import osmnx as ox
import networkx as nx
import utm
import time

from collections import OrderedDict	
import matplotlib.pyplot as plt
from shapely.geometry import Point





class path_generator:

	def __init__(self,current_map, edges):
		rospy.loginfo("Preparing Path")
		self._route_pointx = []
		self._route_pointy = []
		self._graph_proj= current_map
		self._edges= edges
		

		self._NavSatFix= NavSatFix()

		self._path = Path()
		self._path.header.stamp = rospy.Time.now()

		self.route_pub = rospy.Publisher("route_points", Path, queue_size=1)
		rate = rospy.Rate(1)

	def get_start_and_end(self):

		rospy.loginfo("waiting for goal point")
		self.goal_points= rospy.wait_for_message('/move_base_simple/goal', PoseStamped, timeout=None)
		self._endx= self.goal_points.pose.position.x
		self._endy= self.goal_points.pose.position.y
		self._end_point=(self._endy,self._endx)



		rospy.loginfo("waiting for start point")
		gps = rospy.wait_for_message('ada/fix', NavSatFix, timeout=None)

		self._start_lon, self._start_lat = gps.longitude, gps.latitude
		self._start_UTMx, self._start_UTMy, _, _ = utm.from_latlon(self._start_lat, self._start_lon )
		self._start_point=(self._start_UTMy, self._start_UTMx)
		return self._start_UTMx, self._start_UTMy

	def plan_path(self):
		self._startx = self._start_UTMx
		self._starty = self._start_UTMy

		self._origin=(self._starty, self._startx)
		origin_display=(self._startx, self._starty)

		rospy.loginfo("start point from ROSbag: (UTMx,UTMy)")
		rospy.loginfo(origin_display) 
		
		self._destination=(self._endy, self._endx)
		destination_display=(self._endx,self._endy)
		
		rospy.loginfo("Goal Point Entered: (UTMx,UTMy)")
		rospy.loginfo(destination_display) # (434764 4464870)
		
		self._origin_node = ox.get_nearest_node(self._graph_proj, self._origin, method='euclidean')
		self._destination_node = ox.get_nearest_node(self._graph_proj, self._destination, method= 'euclidean')
		self._route = nx.dijkstra_path(G= self._graph_proj, source= self._origin_node,
		 target=self._destination_node , weight='length')
		#ox.plot_graph_route(self._graph_proj, self._route,route_linewidth=6)

		for j in range(0, len(self._edges)):
			if ((self._edges.u[j] == self._route[0]) and 
				(self._edges.v[j] == self._route[1])):

				self._first_edge_geom = self._edges.geometry[j]
			
			if((self._edges.u[j] == self._route[len(self._route)-2]) and 
				(self._edges.v[j] == self._route[len(self._route)-1])):
				
				self._last_edge_geom = self._edges.geometry[j]

		self.first_edge_geometry, self.first_u, self.first_v = self.get_nearest_edge(self._graph_proj, self._start_point)
		self.last_edge_geometry, self.end_v , self.end_u = self.get_nearest_edge(self._graph_proj, self._end_point)

	def generate_path_points(self):
		for r in range(0, len(self._route)-1):
			
			for i in range(0, len(self._edges)-1):
			
				if (self._edges.u[i] == self._route[r]) and (self._edges.v[i] == self._route[r+1]):
			
					for g in range(0, len(self._edges.geometry[i].xy[0])):
						self._route_pointx.append(self._edges.geometry[i].xy[0][g])
						self._route_pointy.append(self._edges.geometry[i].xy[1][g])
		
		self._route_pointx = list(OrderedDict.fromkeys(self._route_pointx))
		self._route_pointy = list(OrderedDict.fromkeys(self._route_pointy))

		self._projected_start_point= self.first_edge_geometry.interpolate(
			self.first_edge_geometry.project(Point(self._startx, self._starty)))


		rospy.loginfo("GPS start point projected to road:")
		rospy.loginfo(self._projected_start_point)

		self._projected_end_point=self.last_edge_geometry.interpolate(
			self.last_edge_geometry.project(Point(self._endx, self._endy)))
		print(self._projected_end_point.x)
		print(self._projected_end_point.y)

		rospy.loginfo("GPS goal point projected to road:")
		rospy.loginfo(self._projected_end_point)

		for i in range (0, len(self._route_pointx)):

			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=i

			if i == 0:
				pose_st.header.stamp=rospy.Time.now()
			 	pose_st.pose.position.x = self._projected_start_point.x
			 	pose_st.pose.position.y = self._projected_start_point.y
			elif i == (len(self._route_pointx)-1):
				pose_st.header.stamp=rospy.Time.now()
				pose_st.pose.position.x = self._projected_end_point.x
				pose_st.pose.position.y = self._projected_end_point.y
			else:
				pose_st.header.stamp=rospy.Time.now()
				pose_st.pose.position.x = self._route_pointx[i]
				pose_st.pose.position.y = self._route_pointy[i]

		 	self._path.poses.append(pose_st)

		self.old_UTMx=self._start_UTMx
		self.old_UTMy=self._start_UTMy

	def publish_path(self):
		self.route_pub.publish(self._path)

	def plot_route_points(self):
		fig, ax = plt.subplots()
		self._edges.plot(ax=ax)
		# self._nodes.plot(ax=ax)
		
		ax.plot(self._startx,self._starty, 'r+')
		ax.plot(self._endx,self._endy, 'r+')


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
	    The last step is to sort the edge segments in ascending order based on the distance
	    from the coordinates to the edge. In the end, the first element in the list of edges
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
	    start_time = time.time()

	    gdf = ox.graph_to_gdfs(G, nodes=False, fill_edge_geometry=True)
	    graph_edges = gdf[["geometry", "u", "v"]].values.tolist()

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

	    print('Found nearest edge ({}) to point {} in {:,.2f} seconds'.format((u, v), point, time.time() - start_time))

	    return geometry, u, v
