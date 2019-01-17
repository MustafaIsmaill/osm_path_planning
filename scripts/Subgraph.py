#!/usr/bin/env python


import urllib
import math
import rospy
import os
import utm
import osmnx as ox

class subgraph:
	def __init__(self, start_x, start_y): 
		self._old_UTMx = start_x
		self._old_UTMy = start_y

		self.url_base = 'https://www.openstreetmap.org/api/0.6/map?bbox='

		self.file_path= os.path.dirname(os.path.abspath(__file__))
		self.file_path_map = self.file_path[:len (self.file_path) -7] + 'maps/'
		self.file_path_subgraph = self.file_path[:len (self.file_path) -7] + 'subgraphs/'

		self.map_load_range = rospy.get_param("grid_map_size")/2

		self.first_time_flag = 1

	def draw_subgraph(self,curr_gps):

		self.curr=curr_gps

		rospy.loginfo("******************")

		self._curr_lat=self.curr.latitude
		self._curr_lon=self.curr.longitude
		if(math.isnan(self.curr.latitude)==False):
			
			self._curr_UTMx, self._curr_UTMy, _, _ = utm.from_latlon(self._curr_lat, self._curr_lon)

	

			# xdiff =self._curr_UTMx - self._old_UTMx
			# ydiff=self._curr_UTMy - self._old_UTMy

			dist = self.calc_distance((self._curr_UTMx, self._curr_UTMy), (self._old_UTMx, self._old_UTMy))
		
			rospy.loginfo("Distance travelled: %f" %dist)

			if self.first_time_flag == 1:
				rospy.loginfo("Generating subgraph")
				self._curr_point=(self._curr_UTMy, self._curr_UTMx)
				# self._curr_node = ox.get_nearest_node(self._graph_proj, self._curr_point, method='euclidean')
				self._old_UTMx=self._curr_UTMx
				self._old_UTMy=self._curr_UTMy
				self.curr_gps_point=(self._curr_lat,self._curr_lon)
				north, south, east, west= ox.bbox_from_point(self.curr_gps_point, distance=self.map_load_range)
				url_name = self.url_base + str(west) + "," + str (south) + "," + str(east) + "," + str(north)
				rospy.loginfo("downloading ...")

				urllib.urlretrieve(url_name, self.file_path_subgraph + 'subgraph.xml')

				self.first_time_flag = 0

			if dist > self.map_load_range:
				
				rospy.loginfo("Generating subgraph")
				self._curr_point=(self._curr_UTMy, self._curr_UTMx)
				self._old_UTMx=self._curr_UTMx
				self._old_UTMy=self._curr_UTMy
				self.curr_gps_point=(self._curr_lat,self._curr_lon)
				north, south, east, west= ox.bbox_from_point(self.curr_gps_point, distance=rospy.get_param("grid_map_size"))
				url_name = self.url_base + str(west) + "%2C" + str (south) + "%2C" + str(east) + "%2C" + str(north)
				rospy.loginfo("downloading ...")

				urllib.urlretrieve(url_name, self.file_path_subgraph + 'subgraph.xml')

			else:
				rospy.loginfo("distance is less than %f meters" %self.map_load_range)
		else:
			rospy.loginfo("gps value is not a number ...")

	def calc_distance(self, point1, point2):
		x1, y1 = point1[0], point1[1]
		x2, y2 = point2[0], point2[1]

		return math.sqrt((x1-x2)**2 + (y1-y2)**2)