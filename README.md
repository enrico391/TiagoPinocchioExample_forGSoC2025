# Simple package for task 3 of project 'Payload visualization and metrics' for GSoC2025

## Usage

Put this package inside workspace of tiago robot then:

```sh
 source install/setup.bash
```

To built the package use:

```sh 
colcon build --packages-select pinocchio_tiago
```

and for use:

```sh
 ros2 run pinocchio_tiago pinocchio_tiago 
```

and publish robot_description with:

```sh 
ros2 launch tiago_description show.launch.py
```



In the srdf folder there is the tiago.srdf file for collision