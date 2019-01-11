road_processing_planning

The package is used Construct paths in street networks from OpenStreetMaps using python and ROS. The package publishes the path waypoints for the vehicle and successive subgraphs.

How it works:

"Configuration Stage"
$ sh initial-config.sh

Run the following commands from the terminal:

1. roslaunch road_processing_planning route_points.launch

2. rosservice call /path_getter + (Goal UTMx, Goal UTMy) 
  example: rosservice call /path_getter 434764 4464870

Note: The package has the maps of Leganes already downloaded and available offline, in case of changing the map, go to line 168 in road_processing_planning/scripts/path_points_processor and change the place_name.

Note: Before changing place_name, make sure that the map desired is valid by going to https://nominatim.openstreetmap.org/ to check map validity.


Package Inputs:

-Goal UTMx

Package Outputs:

1. Path Waypoints (UTMx, UTMy)
2. Subgraphs of path waypoints.

