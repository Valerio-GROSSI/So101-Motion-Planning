# Robot Pose Guidance with MoveIt Motion Planning

This project focuses on ... , with the goal of developing a pipeline to automate ...

## How to Run

1. Calibrate the So101 Follower arm :
```bash
ros2 run so101_robot_hardware my_so101_ros2_calib_executable
```

2. Launch ros2_control and controllers :
```bash
ros2 launch so101_robot_bringup my_so101_robot.launch.py
```

3. Launch MoveIt and RViz interface for motion planning :
```bash
ros2 launch so101_robot_hardware so101_robot_moveit
```

<br>

## Results

<br>

<p align="center">
  <b>So101 Follower Arm</b><br>
  <img src="./so101.gif" width="80%" />
</p>

<br>

<p align="center">
  <b>Sample MoveIt Planning Groups Configuration</b><br>
  <img src="./so101_moveit.gif" width="80%" />
</p>
