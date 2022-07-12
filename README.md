# Control Module

This module uses the position of the robot and the movement commands it receives in order to calculate the velocities for the robot. This module supports 3 basic movements,

- Forward movement
- Backward movement
- Rotational movement

External modules such as Planning module can specify the required movement along with the target coordinates when it makes the request to move the robot.

IF you use this package, Please cite

```sh
@INPROCEEDINGS{Ratnayake2021,
  author={Ratnayake, Kalana and Sooriyaarachchi, Sulochana and Gamage, Chandana},
  booktitle={2021 5th International Conference on Robotics and Automation Sciences (ICRAS)}, 
  title={OENS: An Octomap Based Exploration and Navigation System}, 
  year={2021},
  volume={},
  number={},
  pages={230-234},
  doi={10.1109/ICRAS52289.2021.9476592}}
```
