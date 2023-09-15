# Work on the RB1 robot 
Work done in the **robotics developer masterclass** by The Construct

## RB1 robot modeling in URDF

```
roslaunch my_rb1_description display.launch
```

![imagen](https://github.com/Andy-Leo10/my_rb1_robot/assets/60716487/1232904d-7dcc-4c4a-a158-f0f52cb84d3c)


## Launcher in gazebo with the RB1 robot inside a box

```
roslaunch my_rb1_ros rotate_service.launch
roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch
```

![imagen](https://github.com/Andy-Leo10/my_rb1_robot/assets/60716487/a8301a92-e495-45fb-b89a-2d614ae0d8e9)


## Rotation service for the RB1 robot

```
rosservice call /rotate_robot "degrees: -45"
```

![imagen](https://github.com/Andy-Leo10/my_rb1_robot/assets/60716487/306a5806-fab9-4f55-a5e6-377d2ce21491)
