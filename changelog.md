### July 9th, 2022
- `fake_drone`: add odometry published as a `geometry_msgs/PoseStamped` message
- `local_sensing`
  - add point cloud output in camera frame (to mimic depth camera)
  - refine FOV
  - remove occluded points


## TODO
- [ ] Add basic manual control
- [ ] Mimic gazebo message
- [ ] Improve efficiency
- [ ] OpenCL rendering for local sensing
- [ ] use the same local sensing node for multiple drones

## BUG
- Current framework relies on `poscmd_2_odom` which directly converts position command to odometry and yields quadrotor's SO3 dynamics. Future adjustment should focus on this. 