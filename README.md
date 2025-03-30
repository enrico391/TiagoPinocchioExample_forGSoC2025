# Simple package for task 3 of project 'Payload visualization and metrics' for GSoC2025

## Usage

Put this package inside workspace of tiago robot and start robot description topic:

```sh
 source install/setup.bash
 ros2 launch tiago_description show.launch.py
```

To built the package use:

```sh 
colcon build --packages-select pinocchio_tiago
```

and for use:

```sh
 ros2 run pinocchio_tiago pinocchio_tiago 
```

and then republish robot_description with:

```sh 
ros2 launch tiago_description robot_state_publisher.launch.py
```

In the srdf folder there is the tiago.srdf file for collision

