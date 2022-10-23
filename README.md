# Trajectory tools

Convenience pkg for interfacing with moveit

### use

The launchfiles will default to simulation.  
To run the examples on hardware:

- launch the correct moveit_config bringup.launch

  - without endeffector

  ```shell
  ur10e_moveit_config ur10e_iaac_bringup.launch 
  ```

  - with endeffector

  ```shell
  ur10e_ee_moveit_config ur10e_ee_iaac_bringup.launch 
  ```

- start URcaps on the robot controller

- append the following argument when launching the examples `sim:="false"`
