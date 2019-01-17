#!/usr/bin/env python


import os
import osmnx as ox 
import rospy


class map_processing:

	def __init__(self):
		
		rospy.loginfo("Processing Map")
		
		self.file_path= os.path.dirname(os.path.abspath(__file__))
		self.file_path_map = self.file_path[:len (self.file_path) -7] + 'maps/'
		self.file_path_subgraph = self.file_path[:len (self.file_path) -7] + 'subgraphs/'


	def get_map(self, name):		
		try: 
			self._graph_proj= ox.load_graphml( name +'.xml', folder= self.file_path_map)
		
		except:

			try:
				
				rospy.loginfo("First Downloading Attempt")
				self._graph = ox.graph_from_place(name, network_type='drive')
				self._graph_proj = ox.project_graph(self._graph)
				ox.save_graphml(self._graph_proj ,folder= self.file_path_map , filename=name +'.xml')
			except:
				
				rospy.loginfo("First Downloading Attempt Failed, Retrying Download")
				self._graph = ox.graph_from_address(name, distance=250, network_type='drive')
				self._graph_proj = ox.project_graph(self._graph)
				ox.save_graphml(self._graph_proj ,folder= self.file_path_map ,  filename=name +'.xml')
		
		rospy.loginfo("Map Loaded Successfully")

		return self._graph_proj

	def get_edges(self):
		self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=False, edges=True)

		return self._edges
