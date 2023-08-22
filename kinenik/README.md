# kinenik

This is a project concentrating all the IK (and DK) of the robots at IOC. It tries to contain direct kinematics and inverse kinematics of the robots. Currently _only_ inverse kinematics for Universal Robots is implemented.

## Universal Robot Kinematic

This package provides a closed-form analytical solution for the kinematics of the arm of Universal Robot UR3, UR5, UR10, UR3e, UR5e, UR10e and UR16e robots. To use it, just:

* In your CMakeList.txt add

```
    find_package(kinenik REQUIRED)
    # Extract include directories, because some subsequent
    # catkin packages can't seem to build without this.
    get_target_property(kinenik_INCLUDE_DIRS ${kinenik_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)

    # Link against your superprogram
    target_link_libraries(mysuperprogram  ${kinenik_LIBRARIES}  )
```

* In your code use something similar to this:
```
    # type could be: UR3, UR5, UR10, UR3e, UR5e, UR10e, UR16e
    KinenikUR myrobot(type);
    std::vector<JointPos> theta_sol;
    std::vector<double> values;
    myrobot.solveIK(values[0], values[1], values[2], values[3],  values[4], values[5], values[6], theta_sol);
```
The 'theta_sol' var is a vector of solutions double[6].

lpa 20220512
