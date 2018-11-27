#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx
from road_info_osm_extract.msg import pointsList
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

#added comments

class path_plan:

	def __init__(self, x):
			
			
		self._place_name = 'Universidad Carlos III de Madrid, 30, Avenida de la Universidad'
		
		try:
			self._graph = ox.graph_from_place(self._place_name, network_type='drive')
		except:
			self._graph = ox.graph_from_address(self._place_name, distance=250, network_type='drive')
	
		self._graph_proj = ox.project_graph(self._graph)
		
		self._nodes, self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=True, edges=True)
				
		self._path= Path()
		self._route = []
		
		rospy.init_node('road_processor_planner', anonymous=True, x)
		
		self.route_pub = rospy.Publisher("route_points", Path)
		self._rate = rospy.Rate(1)

		self._startx = self._nodes[:].x[(self._edges[:].u[34])]
		self._starty = self._nodes[:].y[(self._edges[:].u[34])]
		self._endx = self._nodes[:].x[(self._edges[:].v[34])]
		self._endy = self._nodes[:].y[(self._edges[:].v[34])]

		self._origin=(self._startx,self._starty)
		self._destination=(self._endx,self._endy)
		self._origin_node = ox.get_nearest_node(self._graph_proj, self._origin)
		self._destination_node = ox.get_nearest_node(self._graph_proj, self._destination)

		self._route = nx.shortest_path(G= self._graph_proj, source= self._origin_node, target=self._destination_node , weight='length')
		
		route_pointx = []
		route_pointy = []

		# print(len(self._route))
		for r in range(0, len(self._route)-1):
			for i in range(0, len(self._edges)):
				if (self._edges.u[i] == self._route[r]) and (self._edges.v[i] == self._route[r+1]):
					for g in range(0, len(self._edges.geometry[i].xy[0])):
						route_pointx.append(self._edges.geometry[i].xy[0][g])
						route_pointy.append(self._edges.geometry[i].xy[1][g])

		self._path.header.stamp = rospy.Time.now()

		for i in range (0, len(route_pointx)):
			
			pose_st = PoseStamped()
			pose_st.header.stamp=rospy.Time.now()
			pose_st.header.seq=i

		 	pose_st.pose.position.x = route_pointx[i]
		 	pose_st.pose.position.y = route_pointy[i]

		 	self._path.poses.append(pose_st)
		
		while not rospy.is_shutdown():
			print("publishing...")
			self.route_pub.publish(self._path)
			self._rate.sleep()

	def draw_route(self):

		origin=(self._startx, self._starty)
		destination=(self._endx, self._endy)
		fig, ax = ox.plot_graph_route(self._graph_proj, self._route, origin_point=origin, 
			destination_point=destination, save=False, filename='route', file_format='png')

if __name__ == '__main__':
	try:
		path = path_plan()
		# path.draw_route()
	except Exception as e:
		print(e)