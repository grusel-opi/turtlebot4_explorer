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
  Two different methods are developed to record the data captured by the cameras during the execution of the coverage pattern.

- ### remote_recorder
  
  The remote_recorder node contains two action servers for starting and stopping recording images from specified image topics using the rosbag2 package.
  A start request contains a list of image topics to record. When a start request is received, the remote_recorder starts recording via rosbag2 in a sub-process.
  The sender of the request is informed about the discovery and subscription status of these topics via the action feedback topic.
  The action is completed successfully if all topics are discovered and subscribed to.
  If discovery or subscription fails, the action is aborted.
  When a stop request is received, the node sends an interrupt signal to the rosbag2 subprocess and waits for the process to terminate.
  A feedback message informs the sender if messages buffered in memory are still being written.
  When the writing process is finished, the action is completed, and a result message informs us about lost messages.

- ### rosbag2_remote_record_action_interfaces
  
  Defines the actions used in the remote_recorder.

- ### turtlebot4_photographer

  The photographer node captures single images from a list of specified image topics when the trigger signal is received via a ROS topic.
  The images are written directly to the host file system at reached navigation targets.
  A buffer is created for each topic in which the last message received is held.
  When a trigger signal is received, the node waits for a specified time, after which all buffers are locked and decoded.
  The images are stored in the file system in png format.
  The image and trigger callbacks are configured in a reentrant callback group.
  Callbacks from a single reentrant callback group can be executed in parallel when a multithreaded executor executes the node.
  This allows for a delay within the trigger callback to prevent receiving new image messages from being blocked.
  Therefore, this node is executed by a multithreaded executor. The node is also implemented as a composable node.
  This allows it to be loaded with the camera driver nodes to take advantage of ROS 2 intra-process communication.
  
- ### waypoint_plugin
  
  The inputOutputAtWaypoint plugin bridges the photographer node and the navigator and is executed whenever the robot reaches a navigation goal.
  The plugin sends a trigger message to the photographer and waits for a response. Upon receiving a response, the navigator marks the goal as successful.
  If no message is received after a specified time, the plugin raises an error, marking the goal as failed.
  
- ### waypoint_nav
  Hardcoded navigation patterns for test purposes.

  
- ### oakd
  Configuration files an launch scipts for oak-d Pro cameras.
  
- ### imu_estimator
  This is a ROS 2 node that subscribes to the oakd IMU messages in order to estimate the camera's orientation. This is useful for automatically changing the URDF of the robot after mounting the camera at a different angle.
  It uses the [Madgwick AHRS filter](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) for estimation and prints Euler angles.




### References

1. > Matan Keidar and Gal A Kaminka. Robot exploration with fast frontier detection: Theory and experiments. In Proceedings of the 11th International Conference on Autonomous Agents and Multiagent Systems-Volume 1, pages 113–120, 2012
