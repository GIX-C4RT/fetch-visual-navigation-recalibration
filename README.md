# fetch-visual-navigation-recalibration
This package handles the recalibration using ArUco Markermap

# Setup  
build the package as a standard ROS package via 
```
catkin_make
```

first you need to launch the visual navigation stack please check fetch-visual-navigation-stack repo for that  

Execute this command to bring up camera_pose publisher. It will keep publish camera_pose relative to the markermap detected
```
rosrun camera_utils camera_pose
```

Execute this command to bring up markermap static transform publisher. 
```
roslaunch camera_utils static_publisher.launch
```


Execute the following python script by
```
python base_link_marker_map_bind.py
python recalib.py
```
### Note
those script are located in the script folder and since it's not in the package.xml you need to execute them mannually

Now everything should be up and running when you send navigation goals to fetch it will automatically recalibrate itself
