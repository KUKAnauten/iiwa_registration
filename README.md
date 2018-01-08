**REGISTRATION**
Registeres the EEF pose and jointpositions when the "get"-Button on the KUKA SmartPad gets pushed.

1. cd ROS/<catkin_ws>/
2. . devel/setup.bash
3. roscore
4. roslaunch iiwa_registration toolbar.launch → initializes a Toolbar with pushbutton (pose_get) on the KUKA SmartPad. 
5. start ROSSmartServo
6. rosservice call /iiwa/configuration/configureSmartServo „Parameter_see_serviceCall“ → switch to Cartesian Impedance Mode
7. rosrun iiwa_registration button_listener
8. handguide robot endeffector to the desired position and press „get“-button.
9. So far: CartPose and JointPos published to the topics:
    - /iiwa/registered/JointPosition
    - /iiwa/registered/CartesianPose



**serviceCall**
*control_mode: 
                  Position control: "0"
                  Joint impedance : "1"
                  Cartesian impedance: "2"
                  Force control: "3"
                  Sine Force : "4"*

*example for cartesian impedance control("2")*

rosservice call /iiwa/configuration/configureSmartServo "control_mode: 2
joint_impedance:                                                                    
  joint_stiffness: {a1: 0.1, a2: 0.1, a3: 0.1, a4: 0.1, a5: 0.1, a6: 0.1, a7: 0.1}
  joint_damping: {a1: 0.8, a2: 0.8, a3: 0.8, a4: 0.8, a5: 0.8, a6: 0.8, a7: 0.8}
cartesian_impedance:
  cartesian_stiffness: {x: 10.0, y: 10.0, z: 10.0, a: 10.0, b: 10.0, c: 10.0}
  cartesian_damping: {x: 0.7, y: 0.7, z: 0.7, a: 0.7, b: 0.7, c: 0.7}
  nullspace_stiffness: 0.0
  nullspace_damping: 0.7
desired_force: {cartesian_dof: 3, desired_force: 1.0, desired_stiffness: 0.0}
sine_pattern: {cartesian_dof: 0, frequency: 0.0, amplitude: 0.0, stiffness: 0.0}
limits:
  max_path_deviation: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
  max_control_force: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}
  max_control_force_stop: false
  max_cartesian_velocity: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0}" 


**TODO**
  - feed registered jointpositions to moveIt! 
       -> Goal pose --> db??
       -> spawn dummy object?

  - test accuracy of pose
