# Husky Terminal Control
The purpose of the script is to control the Husky through the terminal

It is heavily inspired by https://github.com/kheng-yu/TurtleSim_CPP

The following functions are implemented
1) Specify movement and direction to Husky
2) Specify a specific spot (goal) the Husky should go to

## To use Husky
1. Connect to its WiFi
2. `ssh administrator@192.168.131.1`
3. Voila!

## Transfer files to and from Husky
`scp <directory to file to transfer> <destination directory>`

husky directory would be administrator@192.168.131.1:~/<directorypath>

to transfer to your local directory where terminal is opened or cd to,

`scp administrator@192.168.131.1:~/<directorypath> .`, use -r to move folders
  
# To Do
1. Check Quart2EulerConversion() function, might be the one causing odom issuse
2. Test out in Gazebo
3. Pure Pursuit with slow lap track
