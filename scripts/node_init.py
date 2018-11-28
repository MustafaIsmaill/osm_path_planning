#!/usr/bin/env python

import rospy
import osmnx as ox
import networkx as nx

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from road_processing_planning.srv import *
import time

rospy.init_node('road_processor_planner', anonymous=True)
rospy.spin()