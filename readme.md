# Road_processing_planning

The package is used Construct paths in street networks from OpenStreetMaps using python and ROS. The package publishes the path waypoints whenever it is subscribes to gps readings
as well as producing successive subgraphs of vehicle's location for further processing. 

## Installation

1) Use the package manager [pip](https://pip.pypa.io/en/stable/) to install osmnx.

```bash
pip install osmnx
```
clone the package 
```bash
git clone https://AhmedTarek96@bitbucket.org/lsi/osm_path_planning.git
```

## Running The Packagge

1) open configuration file road_processing_planning/config/params.yaml  to set the map and the grid size wanted.

2) Launch the package.
 ```bash
 roslaunch road_processing_planning route_points.launch 
```
3) Run rqt to specify goal point in UTM.
```bash 
rosrun rqt_publisher  rqt_publisher
```
4) Run rosbag to find starting point in UTM. (without the "")
```bash
rosbag play "bag_name"
```

5) To get grid map for subgraphs launch the ogm_mapping.


## Package Inputs
1) Map.
2) Start and Goal points (UTM).
3) Grid cell size.

## Package Outputs.

1) Publishes waypoints.
2) Generates subgraphs.

