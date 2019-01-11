road_processing_planning

The package is used Construct paths in street networks from OpenStreetMaps using python and ROS. The package outputs the path waypoints whenever service is called
on "/path_getter" topic as well as successive subgraphs of vehicle's location for further processing. 


How it works:

The package depends on osmnx python package, so users need to download it from https://github.com/gboeing/osmnx 

"Configuration Stage"
$ sh initial-config.sh

Run the following commands from the terminal:

1. roslaunch road_processing_planning route_points.launch

2. rosservice call /path_getter + (Goal UTMx, Goal UTMy) 
  example: rosservice call /path_getter 434764 4464870

Note: The package has the maps of Leganes already downloaded and available offline, in case of changing the map, this can be easily done by changing place name variable from the 
configuration file /road_processing_planning/config/document.yaml

Note: Before changing place_name, make sure that the map desired is valid by going to https://nominatim.openstreetmap.org/ to check map validity.


Package Inputs:

-Goal UTMx,UTMy

Package Outputs:

1. Path Waypoints (UTMx, UTMy)
2. Subgraphs of path waypoints.

