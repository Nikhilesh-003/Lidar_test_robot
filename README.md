**Lidar Test Robot**

SOFTWARE USED:
> **OS:** Ubuntu 22.04
> 
> **Framework:** ROS2 Humble
> 
> **Simulation Engine:** Gazebo Classic


This workspace contains a custom_built_robot with Lidar and Depth camera. The node is configured in a way to display the data obtained from the Lidar sensor through the **/scan** topic and also the filtered data having a range of 0 to 120 degrees 
as its field of view and publish this filtered data to a new topic **/filtered_scan**.
