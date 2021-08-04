# Husky Control

These are the packages related to Husky control.
Clone these packages in a workspace, e.g., Husky/src/
then `catkin build && source devel/setup.bash`


## Requirements
Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)

## Husky Terminal Control
for Husky testing using terminal
`roslaunch husky_terminal_control husky_terminal_control`

## mur_common
contains messages used

## path_follower_husky
Pure pursuit controller that tracks given reference points

#### path files (yaml files)
Used to manually publish pathpoints for Husky to follow as this doesnt have a path planner yet.
See path_follower_husky [readme](https://github.com/MURDriverless/HuskyControl/blob/main/path_follower_husky/README.md)


