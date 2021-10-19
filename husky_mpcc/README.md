# Model Predictive Contouring Control
<p align="center">
  <img width="454" height="300" src="https://user-images.githubusercontent.com/78944454/137711457-6073b01d-b5d1-4f3f-905a-70c860ee6a46.gif">
  <img width="300" height="300" src="https://user-images.githubusercontent.com/78944454/137818522-85b7dd92-eb84-40b6-b4f5-c46caab0f031.png">  
</p>

# Disclaimer
This branch is heavily based upon [Alex Liniger's MPCC](https://github.com/alexliniger/MPCC). It is using the version from the fullsize branch, commit 302e12a on 15 Jun 2021.

Main changes:
1. Using Unicycle Model, with Husky's own set of constraints
2. ROS Implementation, with transition code from Slow Lap
3. Slightly faster solve time for Discretization code

## Husky Dynamics
A simple kinematic unicycle model is used in this version
<p align="center">
  <img width="200" height="200" src="https://user-images.githubusercontent.com/78944454/137826421-5731ba0a-432e-43ac-8b5e-f052e6f884a0.png">
</p>

<p align="center">
  <img width="341" height="186" src="https://user-images.githubusercontent.com/78944454/137827369-229ead02-1558-4590-975f-6a32ea3c71b3.png">
  <img width="450" height="186" src="https://user-images.githubusercontent.com/78944454/137827921-4ea7488e-306d-4648-a828-8cb6853f71db.png">
</p>

Note that to simulate controller's ability to deal with unmodelled dynamics, the vlimit is changed to 15m/s in this version. 

To make Husky go fast, go `/opt/ros/<distro>/share/husky_control/config` and edit control.yaml. 

The setting used is max_v = 15, max_a = 10

## Run in ROS, with simple sim
1. Clone this package as a separate package to build
2. Run `sudo ./install.sh` to install dependencies into External folder. If it says file not found, try `sudo chmod +x install.sh`
3. Change directory to reflect the directory you put the files in for `main.cpp` and `Params/config.json`
4. `catkin build` then `source devel/setup.bash`
5. `rosrun husky_mpcc husky_mpcc -0 -1`

## To test EUFS/Small or other track in Gazebo Simulation (Using Husky Sim)
1. Do above steps 1 to 3.
2. Install [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky)
3. Clone this package, then build it with `catkin build`
4. Husky Sim can probably directly load models and world from mursim package, not sure how, so we'll do a bit of moving stuff around. For step 5 to 7, you may need `sudo chown -R user:group directory` to own the directories from ROOT. Replace user:group with your Linux user:group, directory as the folder that contains the file you need to modify or folder you're moving stuff to.
5. Add the following lines to `/opt/ros/<distro>/share/husky_gazebo/package.xml` if they are not there
```
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
  </export>
```
6. Steps below detail how you can import a custom map, example given is the standard eufs_track from [mursim package](https://github.com/MURDriverless/mursim).

6.1. From [mursim package](https://github.com/MURDriverless/mursim), copy `models` folder from `mursim_description` to `/opt/ros/<distro>/share/husky_gazebo/`

6.2. From [mursim package](https://github.com/MURDriverless/mursim), copy `eufs_track.world` or any track you would like to test from `mursim_gazebo/worlds/` to `/opt/ros/<distro>/share/husky_gazebo/worlds`

6.3. `catkin build` again, `source devel/setup.bash` in your workspace folder where you built the package, then `roslaunch husky_mpcc husky_mpcc.launch map:='husky_eufs` and it'll launch the Husky running in eufs_track, currently it defaults to empty world if no map argument is given

6.4. Likewise, you can also do `roslaunch husky_mpcc husky_mpcc.launch map:="trackname"` to try other maps, check the launch file for more details.

### Issues with package building
You may want to try and build the package from scratch, follow these steps.
1. Clone Alex Liniger's MPCC from fullsize branch.
2. Follow his instructions to build MPCC, verify if it works by running `./MPCC`
3. Arrange the folders in the ROS template (all files in src, CMakeList.txt is at same level as src)
4. Clone this repo, replace everything except for External folder.

## To Do
1. Test on more complex tracks.
2. Physical Testing tunings, then test transition in real life.
3. Husky Sim can probably directly load models and world from mursim package, not sure how. Will invetigate if have time.
