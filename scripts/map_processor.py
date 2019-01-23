#!/usr/bin/env python


import os
import osmnx as ox 
import rospy


class map_processing:
	"""
		The map_processing class processes OSM map by downloading it or loading
		if available offline. It also has the ability to extract nodes and edges
		from OSM data by and then saving nodes, edges, and the map itself.
		The map is saved in .xml file. 

		Attributes:
		----------------
		name : str
			this is the name of the map set in the configuration file.
			for example the map now used is leganes,Spain. but it can be easily
			changed.

		Methods
		-------------
		get_map()
			it used to download or load the map offline, if available.
			The loaded map is converted to networkx MultiDiGraph to 
			deal with street networks.

		get_edges()
			returns edges from the map.
	"""




	def __init__(self):
		
		"""

			We set the directory where the the maps and subgraphs should be stored.
			This is done so we can load map if available from this directory,
			or download the map and save it there.
		"""
		rospy.loginfo("Processing Map")
		
		self.file_path= os.path.dirname(os.path.abspath(__file__))
		self.file_path_map = self.file_path[:len (self.file_path) -7] + 'maps/'
		self.file_path_subgraph = self.file_path[:len (self.file_path) -7] + 'subgraphs/'


	def get_map(self, name):		
		
		"""
			This method returns self._graph_proj.It uses try and except to do so.
			First it tries to create object by load the map given map 
			name 'name' added to to the directory. If map not available then tries
			to download it using ox.graph_from_place() method and then project it 
			to UTM system and save it in /maps/ directory. If this download fails,
			then tries to download the map using another method ox.graph_from_address() 
			method and then project it to UTM system and then save it in
			/maps/ directory.


			Returns
			----------
			self._graph_proj : networkx MultiDiGraph
		"""
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
		"""
			Returns edges from the projected graph. OSM graph consist of nodes
			edges.


			Returns:
			---------
			self._edges : geopandas GeoDataFrame	
				self._edges contain roads co-ordinates

		"""
		self._edges = ox.graph_to_gdfs(self._graph_proj, nodes=False, edges=True)

		return self._edges

	