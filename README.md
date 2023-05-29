# HIWI_Programming_Robots

## The link for each task:

- Task1: [main.m](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/Simple_example/main.m#L52)

  - compile: directly run in matlab

- Task2: [task2.m](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/Simple_example/task2.m)

  - compile: directly run in matlab

- Task3 and Task4 : [task_3_4.cpp](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/catkin_ws/src/dqpanda/src/task_3_4.cpp)
  - The plotting script for Task4:  [task4.m](/catkin_ws/task4.m)
  
  - compile: with command  `catkin_make`
  
- Task5: [task_5.cpp](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/catkin_ws/src/dqpanda/src/task5.cpp)

  - compile: with command `catkin_make`

- Task6:[task6.cpp](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/mujoco-2.3.5/task6/task6.cpp)
  - Franka Model comes from[Franka Emika Panda](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
  
  - video: [/task6.mkv](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/task6.mkv)

  - compile: with command `make`
  
- Task7:

  1. Pinocchio:[task7_pinocchio.cpp](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/Task7/task7_pinocchio/task7_pinocchio.cpp)
     - compile with commands:

       ```
       mkdir build
       cd build
       cmake ..
       make
       ```
     
  2. analytical Inverse Kinematics (IK) solver for Franka Emika Panda:[task7_AIK.cpp](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/Task7/task7_AIK/task7_AIK.cpp)
  
     - compile with commands:

       ```
       mkdir build
       cd build
       cmake ..
       make
       ```
  
  - Notes:
  
    - URDF Model comes from [Franka_description](https://github.com/frankaemika/franka_ros/tree/develop/franka_description)
  

    - When converting the Xacro file to URDF file,`gazebo:true` is necessary:
  
      ```
      rosrun xacro xacro panda.urdf.xacro > panda.urdf gazebo:=true
      ```
  
  
    - I meet a problem in this task, please read the [README.md](https://github.com/Fernweh-yang/HIWI_Programming_Robots/blob/main/Task7/README.md)
  
  
  


## Environment

- Ubuntu: 20.04
- Ros: Noetic
- CoppeliaSim: 4.5.1 edu
- dqrobotics : Development PPA
- Matlab: R2023a
- Mujoci: 2.3.5