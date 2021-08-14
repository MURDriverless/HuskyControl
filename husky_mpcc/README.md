# Model Predictive Contouring Control

# Disclaimer
This branch is heavily based upon [Alex Liniger's MPCC](https://github.com/alexliniger/MPCC). It is using the version from the fullsize branch, commit 302e12a on 15 Jun 2021.

## Husky Dynamics
A simple unicycle model is used in this version,

### States
![unicycle](https://user-images.githubusercontent.com/78944454/129431697-3a2fe54c-337d-4d4b-92cb-442be8bf3487.png)

### Vehicle/Actuation Limit
![vlimit](https://user-images.githubusercontent.com/78944454/129431821-b8603b5a-1c20-4a58-87b6-585d4d36f1f6.png)

### Additional Constraints
![w_constraint](https://user-images.githubusercontent.com/78944454/129431906-9bae518b-8d8c-44fb-8de1-f1004c24489f.png)


## Run in ROS
1. Clone this branch as a separate package to build
2. `catkin build` then `source devel/setup.bash`
3. `rosrun husky_mpcc husky_mpcc`

## To test in simulations (Not working yet)
1. Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)
2. Clone this package, then build it with `catkin build`
3. `source devel/setup.bash` then `roslaunch husky_mpcc husky_mpcc.launch`

### Issues with package building
You may want to try and build the package from scratch, follow these steps.
1. Clone Alex Liniger's MPCC from fullsize branch.
2. Follow his instructions to build MPCC, verify if it works by running `./MPCC`
3. Arrange the folders in the ROS template (all files in src, CMakeList.txt is at same level as src)
4. Clone this repo, replace everything except for External folder.

## To Do
1. Check if more complex dynamics needed.
2. Make it work with husky simulator.
3. Make it work in MURsim.
