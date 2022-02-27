# Model Predictive Contouring Control
<p align="center">
  <img width="454" height="300" src="https://user-images.githubusercontent.com/78944454/137711457-6073b01d-b5d1-4f3f-905a-70c860ee6a46.gif">
  <img width="300" height="300" src="https://user-images.githubusercontent.com/78944454/137818522-85b7dd92-eb84-40b6-b4f5-c46caab0f031.png">  
</p>

## Disclaimer
This branch is heavily based upon [Alex Liniger's MPCC](https://github.com/alexliniger/MPCC). It is using the version from the fullsize branch, commit 302e12a on 15 Jun 2021. Do not build this package together with [MURSim](https://github.com/MURDriverless/mursim) or you'll break HuskySim.

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

To reflect the same setup in HuskySim, go `/opt/ros/<distro>/share/husky_control/config` and edit control.yaml. The setting used is max_v = 15, max_a = 10

## Package set up and build
1. Ensure [Husky Simulator Package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky) and [Husky RVIZ package](http://wiki.ros.org/husky_control/Tutorials/Interfacing%20with%20Husky) are installed, take note of your ROS distro (Ubuntu 18.04 will use Melodic). 
2. Clone this package into your workspace/src folder, you may skip this step if you have cloned the entire HuskyControl package.
3. Run `sudo ./install.sh` to install dependencies into External folder. If it says file not found, try `sudo chmod +x install.sh` first.
4. Change directory in `main.cpp` and `Params/config.json` to reflect the directory you put the files in. Use `pwd` in terminal to get the full directory of the folder you're in.
5. `catkin build` and it should work. Remember to `source devel/setup.bash` so your terminal can find the new packages. You may need to run multiple `catkin build` before it successfully builds due to dependencies and/or error of previous packages.

### Issues with package building
If you run into issues when building packages, try build it again, sometimes it requires multiple builds due to dependencies on other packages. 

If it still doesn't work, it might be because of the external libraries. Please replace everything in the External folder with [these](https://drive.google.com/drive/u/1/folders/16xUVZtKH77O1hIqJbc8R4slYYIPgZIeA).

If that still didn't work, you may want to try and build the package from scratch, follow these steps.
1. Clone Alex Liniger's MPCC from fullsize branch.
2. Follow his instructions to build MPCC, verify if it works by running `./MPCC`
3. Arrange the folders in the ROS template (all files in src, CMakeList.txt is at same level as src)
4. Clone this repo, replace everything except for External folder.

## Run in ROS, with SimpleSim
1. Run `roscore` in a new terminal.
2. In the terminal that you ran `source devel/setup.bash` on, do
```
rosrun husky_mpcc husky_mpcc -0 -1
```
3. Alternatively, if you want to use a roslaunch command directly and skip running `roscore` in a new terminal, you can use the following command. Look at launch file for more details, but essentially the arguments sets Gazebo and RVIZ to not run, the controller to not communicate with SLAM system, and to skip directly to Fast Lap.
```
roslaunch husky_mpcc husky_mpcc.launch gazebo:=0 rviz:=0 comm:=0 skip:=1
```

## Run full Autonomous Control Pipeline with HuskySim
1. Ensure the whole HuskyControl package is able to build. **You may need to build HuskyControl multiple times before it successfully builds everything.** Refer to specific package README for instructions if you face issues.
2. Run the following to begin. It'll launch HuskySim in Gazebo and RViz.
```
roslaunch husky_mpcc slow2fast.launch
``` 
3. Open a new terminal at the base workspace, `source devel/setup.bash` then run the following command to simulate cone position input from SLAM. For reference, if you opened the new terminal in the base workspace folder, you can replace `<directory to mapname.yaml>` with `src/HuskyControl/race_tracks/cones_big_map.yaml` to publish cone positions of the FSG 2018 race track.
```
rostopic pub /mur/slam/true_cones mur_common/cone_msg -f <directory to mapname.yaml>
``` 
4. Voila, it works!

## To test EUFS/Small or other track in Gazebo Simulation (Using Husky Sim)
1. Husky Sim can probably directly load models and world from mursim package, not sure how, so we'll do a bit of moving stuff around. For step 5 to 7, you may need `sudo chown -R user:group directory` to own the directories from ROOT. Replace user:group with your Linux user:group, directory as the folder that contains the file you need to modify or folder you're moving stuff to.
2. Add the following lines to `/opt/ros/<distro>/share/husky_gazebo/package.xml` if they are not there
```
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
  </export>
```
3. Steps below detail how you can import a custom map, example given is the standard eufs_track from [mursim package](https://github.com/MURDriverless/mursim).

3.1. From [mursim package](https://github.com/MURDriverless/mursim), copy `models` folder from `mursim_description` to `/opt/ros/<distro>/share/husky_gazebo/`

3.2. From [mursim package](https://github.com/MURDriverless/mursim), copy `eufs_track.world` or any track you would like to test from `mursim_gazebo/worlds/` to `/opt/ros/<distro>/share/husky_gazebo/worlds`

4. `catkin build` again, `source devel/setup.bash` in your base workspace folder where you built the mpcc package, then `roslaunch husky_mpcc husky_mpcc.launch map:='husky_eufs'` and it'll launch the Husky running in eufs_track, currently it defaults to empty world if no map argument is given.

4.1. Likewise, you can also do `roslaunch husky_mpcc husky_mpcc.launch map:="trackname"` to try other maps, check the launch file for more details.

## To Do
1. Test on more complex tracks.
2. Physical Testing tunings, then test transition in real life.
3. Husky Sim can probably directly load models and world from mursim package, not sure how. Will invetigate if have time.
