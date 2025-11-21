## ROS 2 Colcon workspace for my thesis "Autonomous Data Acquisition for 3D Reconstruction".

This workspace contains four ROS 2 packages:

- ### turtlebot4_explorer

  Main package containing logic for exploration and coverage of an environment using RGB(-D) camera(s). Exploration and coverage are subsequent stages but can be used independently. 
  The exploration uses the Wavefront Frontier Detector algorithm [1] to acquire a 2D map of the environment (using e.g. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/)).
  The coverage uses the obtained 2D map to capture data of the environment and implements two strategies. The first strategy, named the boundary acquisition method, places positions at the boundary between free space and occupied space.
  By driving along this boundary with all cameras facing toward the occupied space, each obstacle is covered with overlapping image data. The free space is covered by driving along the same path in the opposite direction a second time.
  The second strategy, named the starpose acquisition strategy, evenly distributes positions in the accessible free space and uses multiple orientations for each position as acquisition poses.
  Both strategies aim to achieve a successful [Structure from Motion](https://en.wikipedia.org/wiki/Structure_from_motion) reconstruction and a [Gaussian splatting](https://de.wikipedia.org/wiki/Gaussian_Splatting) scene
  through sufficient overlap and complete coverage of the environment with image data. The robot is controlled using [Navigation2](https://github.com/ros-navigation/navigation2).

- ### turtlebot4_photographer
- ### waypoint_plugin
- ### waypoint_nav




### References

1. > Matan Keidar and Gal A Kaminka. Robot exploration with fast frontier detection: Theory and experiments. In Proceedings of the 11th International Conference on Autonomous Agents and Multiagent Systems-Volume 1, pages 113–120, 2012
