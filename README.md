# Husky Control

These are the packages related to Husky control.

Clone these packages in a workspace, e.g., Husky/src/

then `catkin build && source devel/setup.bash`

To launch specific packages, refer to that package's README.

## Requirements
Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)

## To use with Husky, remote control with Master/Slave
1. Connect to its WiFi
2. If this is your first time doing this, you'll likely need to set up the workspace, refer to Step 7: Setting up Your Workspace in "Husky_getStarted.pdf", which is in the Guide folder.
3. 
```
export ROS_MASTER_URI=http://cpr-umb02:11311
export ROS_HOSTNAME=<your pc hostname>
```
4. run `rostopic list` and you should see Husky's ROS topics, else check connection or do step 2.
5. `roslaunch husky_viz view_robot.launch`
6. source your workspace/devel/setup.bash, and you can run your own package to interact with Husksy. 
7. If using `roslaunch`, make sure Gazebo and RVIZ is not set to launch together! Typically you'll have to `gui:=false rviz:=false` when launching package

## Transfer files to and from Husky
```
ssh administrator@192.168.131.1`
scp <directory to file to transfer> <destination directory>`
```

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
