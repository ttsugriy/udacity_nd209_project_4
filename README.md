A ROS simulation of a 2-wheel robot equipped with a camera and a LiDAR that can chase a white ball if it's in its view. Project consists of 2 catkin packages:
 - `my_robot` - contains models and their assets
 - `ball_chaser` - contains code for
   - `drive_bot` node that provides a service for moving robot around
   - `process_image` node that processes camera images, identifies white ball and sends commands to `drive_bot` to move towards it
   
In order to run the simulation, you need to setup catkin workspace:
```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
catkin_ws/src$ catkin_init_workspace
```
and checkout this repository into its `src` directory
```bash
catkin_ws/src$ git clone https://github.com/ttsugriy/udacity_nd209_project_2.git .
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

To launch the `drive_bot` and `process_image`

```bash
catkin_ws/src$ source devel/setup.bash $ roslaunch ball_chaser ball_chaser.launch
```

In order to see camera's images you can run
```bash
catkin_ws/src$ source devel/setup.bash $ rosrun rqt_image_view rqt_image_view
```