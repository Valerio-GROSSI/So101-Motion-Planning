# Robot Pose Guidance with MoveIt Motion Planning

This project focuses on ... , with the goal of developing a pipeline to automate ...

<br>

## How to Run

1. To have a python environment containing the expected libraries for running the scripts :
```bash
conda create -n name python=3.10
conda activate name
pip install -r requirements.txt
```

2. Download MOT17 dataset, available with this command :
```bash
ros2 run so101_robot_hardware my_so101_ros2_calib_executable
```

3. Move MOT17 into Inputs folder :
```bash
ros2 launch so101_robot_bringup my_so101_robot.launch.py```

4. Execute the command :
```bash
ros2 launch so101_robot_hardware so101_robot_moveit```

<br>

## Sample Results

<p align="center">
  <img src="./so101.gif" width="80%" />
</p>

<p align="center">
  <img src="./so101_moveit.gif" width="80%" />
</p>

README is still a work in progress.
