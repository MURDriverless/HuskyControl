# Husky Control

These are the packages related to Husky control.

Clone these packages in a workspace, e.g., Husky/src/

then `catkin build && source devel/setup.bash`

To launch specific packages, refer to that package's README.

## Requirements
Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)

## To use with Husky
1. Connect to its WiFi
2. `ssh administrator@192.168.131.1`
3. Transfer the husky_terminal_control package into Husky
4. `catkin_make` to build

## Transfer files to and from Husky
`scp <directory to file to transfer> <destination directory>`

husky directory would be administrator@192.168.131.1:~/<directorypath>

to transfer to your local directory where terminal is opened or cd to,

`scp administrator@192.168.131.1:~/<directorypath> .`, use -r to move folders

## Husky Terminal Control
To test Husky using Terminal
`roslaunch husky_terminal_control husky_terminal_control`

## mur_common
Contains ROS msgs that MUR uses

## path_follower_husky
Pure pursuit controller that tracks given reference points

#### path files (yaml files)
Used to manually publish pathpoints for Husky to follow as this doesnt have a path planner yet.
  
See path_follower_husky [readme](https://github.com/MURDriverless/HuskyControl/blob/main/path_follower_husky/README.md)
  
# To Do
1. Pure Pursuit with slow lap track
2. Backup whole drive of Husky
3. ROS Master/Slave setup so won't mess up Husky's original files
