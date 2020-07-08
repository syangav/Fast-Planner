# Fast-Planner

## Overview

**Fast-Planner** is a robust and computationally efficient planning system that enables quadrotor fast flight in complex unknown environments.
It contains a collection of carefully designed techniques:

- Kinodynamic path searching
- B-spline-based trajectory optimization
- Topological path searching and path-guided optimization


### File Structure

Key modules are contained in __fast_planner__ and a lightweight __uav_simulator__ is used for testing. Key components of __fast_planner__ are:

- __plan_env__: The online mapping algorithms. It takes in depth image (or point cloud) and camera pose (odometry) pairs as input, do raycasting to update a probabilistic volumetric map, and build an Euclidean signed distance filed (ESDF) for the planning system. 
- __path_searching__: Front-end path searching algorithms. 
  Currently it includes a kinodynamic path searching that respects the dynamics of quadrotors.
  It also contains a sampling-based topological path searching algorithm to generate multiple topologically distinctive paths that capture the structure of the 3D environments. 
- __bspline__: A implementation of the B-spline-based trajectory representation.
- __bspline_opt__: The gradient-based trajectory optimization using B-spline trajectory.
- __active_perception__: Perception-aware planning strategy, to appear.
- __plan_manage__: High-level modules that schedule and call the mapping and planning algorithms. Interfaces for launching the whole system, as well as the configuration files are contained here.


## 1. Prerequisites

- Our software is developed and tested in Ubuntu 18.04, melodic. 

- We use [**NLopt**](https://nlopt.readthedocs.io/en/latest/NLopt_Installation) to solve the non-linear optimization problem.

- The __uav_simulator__ depends on the C++ linear algebra library __Armadillo__, which can be installed by ``` sudo apt-get install libarmadillo-dev ```.

- _Optional_: If you want to run the more realistic depth camera in __uav_simulator__, installation of [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) is needed. Otherwise, a less realistic depth sensor model will be used (See section _Use GPU Depth Rendering_ below).

## 2. Build on ROS

After the prerequisites are satisfied, you can clone this repository to your catkin workspace and catkin_make. A new workspace is recommended:
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/syangav/Fast-Planner.git
  cd ../
  catkin_make
```

### Use GPU Depth Rendering (Optional)

 The **local_sensing** package in __uav_simulator__ has the option of using GPU or CPU to render the depth sensor measurement. By default, it is set to CPU version in CMakeLists:
 
 ```
 set(ENABLE_CUDA false)
 # set(ENABLE_CUDA true)
 ```
However, we STRONGLY recommend the GPU version, because it generates depth images more like a real depth camera.
To enable the GPU depth rendering, set ENABLE_CUDA to true, and also remember to change the 'arch' and 'code' flags according to your graphics card devices. Both set to 75 work at least for GTX 1650. 

```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_75,code=sm_75;
    ) 
``` 

If ENABLE_CUDA = true, depth_topic = /pcl_render_node/depth = /sdf_map/depth --> image (640, 480) with depth is used, singal channel 
In file pcl_render_node.cpp, function render_currentpose is called periodically (always). pub_depth publish depth_topic, pub_color publish the same depth image in rainbow color for visualization. Depending on the received pose type (pose/odom), SDFMap::depthPoseCallback or SDFMap::depthOdomCallback is called with pose & depth image. The md_.depth_image_ is updated accordingly. In SDFMap::updateOccupancyCallback, projectDepthImage & raycastProcess, the md_.occupancy_buffer_ is updated by md_.depth_image_. In SDFMap::updateESDFCallback, updateESDF3d is called. 

If ENABLE_CUDA = false, cloud_topic / /pcl_render_node/cloud = /sdf_map/cloud --> point cloud with 180 degrees heading is used 
In file pointcloud_render_node.cpp, function renderSensedPoints is called periodically. If the current odom point to the nearest obstacle is within horizon (using kdtree), then a pointcloud message will be published with information from 180 degrees heading forwards. sdf_map.cpp will receive this local point cloud in SDFMap::cloudCallback and store the data in md_.occupancy_buffer_inflate_ for later usage. 

In either cases, the function SDFMap::visCallback is called periodically to visualize two things in rviz: local point cloud publishMap and local point cloud inflated publishMapInflate(false). 

## 3. Run the Simulation

Run [Rviz](http://wiki.ros.org/rviz) with our configuration firstly:

```
  <!-- go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage rviz.launch
```

Then run the quadrotor simulator and __Fast-Planner__. 
Several examples are provided below:

### 3.1 Kinodynamic Path Searching & B-spline Optimization

In this method, a kinodynamic path searching finds a safe, dynamically feasible, and minimum-time initial trajectory in the discretized control space. 
Then the smoothness and clearance of the trajectory are improved by a B-spline optimization.
To test this method, run:

```
  <!-- open a new terminal, go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage kino_replan.launch
```

Normally, you will find the randomly generated map and the drone model in ```Rviz```. At this time, you can trigger the planner using the ```2D Nav Goal``` tool. When a point is clicked in ```Rviz```, a new trajectory will be generated immediately and executed by the drone. A sample is displayed below:

<!-- add some gif here -->
 <p align="center">
  <img src="files/ral19_3.gif" width = "480" height = "270"/>
 </p>


### 3.2 Topological Path Searching & Path-guided Optimization

This method features searching for multiple trajectories in distinctive topological classes. Thanks to the strategy, the solution space is explored more thoroughly, avoiding local minima and yielding better solutions.
Similarly, run:

```
  <!-- open a new terminal, go to your workspace and run: -->
  source devel/setup.bash
  roslaunch plan_manage topo_replan.launch
```

then you will find the random map generated and can use the ```2D Nav Goal``` to trigger the planner:

<!-- add some gif here -->
 <p align="center">
  <img src="files/icra20_3.gif" width = "480" height = "270"/>
 </p>


## 4. Use in Your Application

