# ECN Baxter

This package is made on ROS 2. Its goal is to simplify the use of the Baxter robot 
for the robotics programming courses in Centrale Nantes.

This package was made by the student team **Baxterminator** :

- **_Geoffrey CÔTE_**: main maintainer (especially component-related nodes and game nodes)
- **_Thimothée Corroëne_**: maintainer of the vision part
- **_Chira De-Saint-Giniez_** : maintainer of the simulation


## Robot effectors

This packages actually provide a node that simplify the use of the two grippers of 
the robot. It automates the calibration and the auto-release of the vacuum gripper.

This simplification use a new topic and a new message (ecn_baxter::msg::BaxterAction)
```YAML
Topic : /baxter/action

#######################
# Message description #
#######################
string component
string action
```

The two values for the components are *"left_gripper"* and *"right_gripper"* and the 
two main action are *"grip"* and *"release"*.

But before that, a node should be launched for each gripper, so to launch both grippers :
```YAML
ros2 run ecn_baxter gripper_node --ros-arg -p gripper_side:=right
ros2 run ecn_baxter gripper_node --ros-arg -p gripper_side:=left
```

## Game Competition

This package also aims to add basic nodes to create games.

The main node to use is the GameMaster. Its goal is to make enslave the bridges to allow 
two designated persons to publish from ROS2 at the time. It also adds basic functionality
to show timers and points as well as initializing the games properties.