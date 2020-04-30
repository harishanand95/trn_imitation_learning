# trn_imitation_learning

##### Required other ROS packages: 
- PX4 Firmware (https://github.com/PX4/Firmware/)
- MAVROS (https://github.com/mavlink/mavros)
- sitl_gazebo (https://github.com/PX4/sitl_gazebo/)

#### Setup
Add baylands model to gazebo path. (Baylands model is from OSRF latest gazebo commit)

#### How to gather data in Gazebo PX4 world
` python data_gather.py` will hover the UAV at the start location at height 5m.

`python tele_operation.py` will let us move the vehicle along the terrain.

`rosrun  trn_imitation_learning read_bag` to write images to the folder specified in code. Update the folder location and do a catkin build.

The following keys are the mapped for desired effect:

                1. Left key - starts moving the drone forward by 1m/s.
                2. Up - increase z velocity by 0.5 m/s
                3. Down - decrease z velocity by 0.5m/s
                4. Shift - go back to initial start location
                5. Esc - quit recording keys (EXIT)

#### Learning code 
Use `src/write_tf_record.ipynb` and `src/imitation_learning.ipynb` to learn UP, DOWN and STAY actions for the drone from depth images saved from `rosrun  trn_imitation_learning read_bag`.

#### Inference

- Run `roslaunch trn_imitation_learning mavros_posix_sitl.launch` to launch the gazebo world.
- Run `rosrun trn_imitation_learning data_gather` to publish `mono8` depth images.
- Run `python nn_controller.py` and `python imitation_learning.py` to start the controller code and key mapping to enable nn control.
- Press <left key> to switch to learned controller.
  
#### Video 

[![https://www.youtube.com/watch?v=RQT57leu2-E](https://img.youtube.com/vi/RQT57leu2-E/0.jpg)](https://www.youtube.com/watch?v=RQT57leu2-E)
#### Screenshot of the world

Urban Environment
![alt text](worlds/1.jpg)

Bayland Environment
![alt text](worlds/2.jpg)
