## Navigation Wheelchair



### Installation
At first to start the system we need to install the move_base package
```
sudo apt install ros-noetic-navigation
```

With this package includes ``amcl``, ``costmap_2d``, ``move_base``, ``dwa_local_planner`` ... and many more.

For more details you can use the following link
http://wiki.ros.org/navigation?distro=noetic

With this some planning algorithms are already installed and can be used like ``navfn``, ``global_planner`` and ``carrot_planner`` for global planning 
and ``dwa_local_planner`` and ``base_local_planner`` for local planning.



Inside our wheelchair we also want to test the ``teb_local_planner`` with its time elastic band methode. 

```
sudo apt install ros-noetic-teb-local-planner
```

Inside the simulation file we also use ``ira_laser_tools`` for merging two laser scan data
```
sudo apt install ros-noetic-ira-laser-tools
```

### Launch
The lauch of the simulation file:
```
roslaunch navigation navigation_nursing_home_sim.launch
```
The amcl lauch file will be automatically started inside ``navigation_nursing_home_sim.launch``. 
A premade map of a replicated nursing home will be launched inside of gazebo. For better overview of all the data that will be sent rviz is also going to be opened.

### Planner
Inside the params Folder you can change params of the costmaps, some global_planner and some local_planner.

global_planner:

- carrot_global_planner (just a straight line from point A to point B)
- global_global_planner (Extension of navfn with possibility to use A* in addition to Dijkstra)
- navfn_global_planner (uses Dijkstra)

local_planner:

- dwa_local_planner (Dynamic Window Approach)
- trajectory_local_planner (Dynamic Window Approach)
- tep_local_planner (Time Elastic Band)
