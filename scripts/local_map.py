#!/usr/bin/env python


import urllib
import math
import rospy
import os
import utm
import osmnx as ox

class local_map:

	"""
	This class generates local_map every time the vehicle moves a certain distance
	set by the user in the configuration file. The length and width of the generated
	local_map are twice the the distance moved. 

	Parameters:
	-------------
	start_x : sensor_msgs.msg._NavSatFix.NavSatFix
		
		This is start GPS Latitude converted to UTM easting

	start_y : sensor_msgs.msg._NavSatFix.NavSatFix
		This is start GPS Longitude converted to UTM northing

	Methods:
	-----------
	draw_local_map()

		Recieves the current GPS co-ordinates of the vehicle and 
		converts them to UTM, then checks if the car moved a certain
		distance pre set by the user in configuration file and generates
		local map of the vehcile if this distance is exceeded.
		
	Functions:
	-----------
	calc_distance()

		returns euclidean distance between to points
	"""
	
	def __init__(self, start_x, start_y): 
		"""
		
	Parameters:
	-------------
	start_x : sensor_msgs.msg._NavSatFix.NavSatFix
		
		This is start GPS Latitude converted to UTM easting

	start_y : sensor_msgs.msg._NavSatFix.NavSatFix
		This is start GPS Longitude converted to UTM northing
		
		"""
		self._old_UTMx = start_x
		self._old_UTMy = start_y

		self.url_base = 'https://www.openstreetmap.org/api/0.6/map?bbox='

		self.file_path= os.path.dirname(os.path.abspath(__file__))
		self.file_path_map = self.file_path[:len (self.file_path) -7] + 'maps/'
		self.file_path_local_map = self.file_path[:len (self.file_path) -7] + 'local_maps/'

		self.map_load_range = rospy.get_param("grid_map_size")/2

		self.first_time_flag = 1

	def draw_local_map(self,curr_gps):

		"""
			generates subgrah by recieving vehicle GPS points. First the
			Nan GPS points are excluded. Then these GPS points are converted
			to UTM co-ordinates and the distance between Current point and 
			starting point is calculated and the flag=1 at beginning so
		 	the current GPS point is considered as old point and the bounding
		 	box is generated from this point to be passed as arguement for
 			downloading the local_map.The local_map is then downloaded from OSM API, 
 			and the flag is set to 0. This to generate a local_map as soon as the first
 			gps Reading is recieved.
			
			The next local_maps are generated every time the vehicle moves a certain
			distance by measuring distance between the old GPS and current GPS
			points.  If this distance is exceed, the current GPS is set to be old
			Point, and the a local_map is downloaded and stored as .xml file 
			in the same way.

			Parameters:
			
			curr_gps : sensor_msgs.msg._NavSatFix.NavSatFix
				The gps point recieved from ROS Topic


		"""
		

		self.curr=curr_gps

		rospy.loginfo("******************")

		self._curr_lat=self.curr.latitude
		self._curr_lon=self.curr.longitude
		if(math.isnan(self.curr.latitude)==False):
			
			self._curr_UTMx, self._curr_UTMy, _, _ = utm.from_latlon(self._curr_lat, self._curr_lon)


			dist = self.calc_distance((self._curr_UTMx, self._curr_UTMy), (self._old_UTMx, self._old_UTMy))
		
			rospy.loginfo("Distance travelled: %f" %dist)

			if self.first_time_flag == 1:
				rospy.loginfo("Generating local_map")
				self._curr_point=(self._curr_UTMy, self._curr_UTMx)
				self._old_UTMx=self._curr_UTMx
				self._old_UTMy=self._curr_UTMy
				self.curr_gps_point=(self._curr_lat,self._curr_lon)
				north, south, east, west= ox.bbox_from_point(self.curr_gps_point, distance=rospy.get_param("grid_map_size"))
				url_name = self.url_base + str(west) + "," + str (south) + "," + str(east) + "," + str(north)
				rospy.loginfo("downloading ...")

				urllib.urlretrieve(url_name, self.file_path_local_map + 'local_map.xml')

				self.first_time_flag = 0

			if dist > self.map_load_range:
				
				rospy.loginfo("Generating local_map")
				self._curr_point=(self._curr_UTMy, self._curr_UTMx)
				self._old_UTMx=self._curr_UTMx
				self._old_UTMy=self._curr_UTMy
				self.curr_gps_point=(self._curr_lat,self._curr_lon)
				north, south, east, west= ox.bbox_from_point(self.curr_gps_point, distance=rospy.get_param("grid_map_size"))
				url_name = self.url_base + str(west) + "%2C" + str (south) + "%2C" + str(east) + "%2C" + str(north)
				rospy.loginfo("downloading ...")

				urllib.urlretrieve(url_name, self.file_path_local_map + 'local_map.xml')

			else:
				rospy.loginfo("distance is less than %f meters" %self.map_load_range)
		else:
			rospy.loginfo("gps value is not a number ...")

	def calc_distance(self, point1, point2):
		"""
		"""

		x1, y1 = point1[0], point1[1]
		x2, y2 = point2[0], point2[1]

		return math.sqrt((x1-x2)**2 + (y1-y2)**2)