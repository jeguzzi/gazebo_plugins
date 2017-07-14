# gazebo_plugins

This plugin publishes three topics:
- `~/joint_states`: a [sensor_msgs/JointState.msg](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html) with complete information, i.e. angle, angular speed and effort (torque) for all wheel joints;
- `~/contact_state`: a [ContactState.msg](https://github.com/jeguzzi/gazebo_plugins/blob/master/msg/ContactState.msg) that informs you if the wheels (or the chassis) are touching the terrain;
- `~/power`: a
[Power.msg](https://github.com/jeguzzi/gazebo_plugins/blob/master/msg/Power.msg) that contains (motor) power and consumed energy.

Note that the pioneer3at simulation has no motors but a controller that instantaneously change the joint's angular velocity. We estimate *mechanical* power by computing for each wheel w in {fl = front left, fr, bl, br}

  ![equation](http://latex.codecogs.com/png.latex?P_w%3D%5Comega%5Ccdot%5Ctau)  

and then

![equation]( http://latex.codecogs.com/png.latex?P%3D%5Ctheta(P_{fl}+P_{bl}%29+%5Ctheta(P_{fr}+P_{br}%29 )

by assuming that power is shared by pairs of motors on the same side and that not power can be generated by braking.

## Urdf model

Take a look at the [model](https://github.com/jeguzzi/gazebo_plugins/blob/master/model/pioneer3at.urdf).

To enable the plugin, you have to add:
```xml
<gazebo>
    <plugin name="gazebo_ros_traversability" filename="libgazebo_ros_traversability.so">
      <robotNamespace>sim_p3at</robotNamespace>
      <jointName>p3at_front_left_wheel_joint, p3at_front_right_wheel_joint, p3at_back_left_wheel_joint, p3at_back_right_wheel_joint</jointName>
      <linkName>p3at_front_left_wheel, p3at_front_right_wheel, p3at_back_left_wheel, p3at_back_right_wheel, base_link</linkName>
      <updateRate>100.0</updateRate>
      <alwaysOn>true</alwaysOn>
    </plugin>
</gazebo>
<gazebo reference="p3at_front_left_wheel">
  <sensor name="cfl" type='contact'>
  <contact>
    <collision>p3at_front_left_wheel_collision</collision>
  </contact>
  </sensor>
</gazebo>
<gazebo reference="p3at_back_left_wheel">
  <sensor name="cbl" type='contact'>
  <contact>
    <collision>p3at_back_left_wheel_collision</collision>
  </contact>
  </sensor>
</gazebo>
<gazebo reference="p3at_front_right_wheel">
  <sensor name="cfr" type='contact'>
  <contact>
    <collision>p3at_front_right_wheel_collision</collision>
  </contact>
  </sensor>
</gazebo>
<gazebo reference="p3at_back_right_wheel">
  <sensor name="cbr" type='contact'>
  <contact>
    <collision>p3at_back_right_wheel_collision</collision>
  </contact>
  </sensor>
</gazebo>
<gazebo reference="base_link">
  <sensor name="blc" type='contact'>
  <contact>
    <collision>base_link_collision</collision>
  </contact>
  </sensor>
</gazebo>
<gazebo reference="top_plate">
  <sensor name="tpc" type='contact'>
  <contact>
    <collision>top_plate_collision</collision>
  </contact>
  </sensor>
</gazebo>
```
to your model. The plugin will publish information about all joints in `<jointName>` and all collisions of links in `<linkName>`. To compute power, we assume that joints are listed in the following order:
1. front left wheel
1. front right wheel
1. back_left_wheel
1. back_right_wheel


## Demo

Attach a joystick and try to run the demo

```roslaunch gazebo_traversability_plugin demo.launch```

Deadman button is the left trigger. Right vertical axis controls linear speed; left horizontal axis controls angular speed.
