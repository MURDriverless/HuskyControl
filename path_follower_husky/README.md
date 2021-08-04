# HUSKY PATH FOLLOWER

Husky basic control for tracking path points
![huskygif](https://user-images.githubusercontent.com/75785603/126956952-6ba4b37d-5abd-46a7-b392-22ab6943f270.gif)


## launch
* contains launch file
* launches both gazebo and rviz
```
source devel/setup.bash
roslaunch path_follower_husky follower_husky.launch
```

## rviz
* contains rviz config
* marks the path and trajectory of Husky


### Publishing path points

The path points are published by the path planner, however, for this case, we will manually publish path points from a yaml file.

open a new terminal (same dir)
```
source devel/setup.bash
rostopic pub -r 2 /mur/planner/path mur_common/path_msg -f pathh.yaml
```

