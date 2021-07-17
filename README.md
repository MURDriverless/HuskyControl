# Husky Terminal Control
The purpose of the script is to control the Husky through the terminal

It is heavily inspired by https://github.com/kheng-yu/TurtleSim_CPP

The following functions are implemented
1) Specify movement and direction to Husky
2) Specify a specific spot (goal) the Husky should go to

## To test in simulations
1. Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)
2. Clone this package, then build it with `catkin build`
3. `source devel/setup.bash` then `roslaunch husky_terminal_control husky_terminal_control.launch`

## To use with Husky
1. Connect to its WiFi
2. `ssh administrator@192.168.131.1`
3. Transfer the husky_terminal_control package into Husky
4. `catkin_make` then `rosrun husky_terminal_control husky_terminal_control`

## Transfer files to and from Husky
`scp <directory to file to transfer> <destination directory>`

husky directory would be administrator@192.168.131.1:~/<directorypath>

to transfer to your local directory where terminal is opened or cd to,

`scp administrator@192.168.131.1:~/<directorypath> .`, use -r to move folders
  
# To Do
Pure Pursuit with slow lap track
ROS Master/Slave setup so won't mess up Husky's original files
