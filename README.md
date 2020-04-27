A ROS simulation of a 2-wheel robot equipped with a camera and a LiDAR that is localizing its position using AMCL.
 - `my_robot` - contains models and their assets
   
In order to run the simulation, you need to setup catkin workspace:
```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
catkin_ws/src$ catkin_init_workspace
```
and checkout this repository into its `src` directory
```bash
catkin_ws/src$ git clone https://github.com/ttsugriy/udacity_nd209_project_3.git .
```

Once workspace and sources are in place, the project can be built using `catkin`:
```bash
catkin_ws/src$ catkin_make
```

To launch the gazebo world run
```bash
catkin_ws/src$ source devel/setup.bash
catkin_ws/src$ roslaunch my_robot world.launch
```

To launch AMCL:
```
catkin_ws/src$ roslaunch my_robot amcl.launch
```

To launch teleop package that allows controlling the robot
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```