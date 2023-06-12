# Robotics 2조팀플


## Usage


```bash
source catkin_ws/devel/setup.bash
```


#### launch World + box random moving
```bash
roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch
```


#### Environment Node + Conflict Regenerate
```bash
rosrun agent_node env.py
```










***
## Agent Node


#### Subscribed Topics


* **Published Topics**
    * `/cmd_vel` (`geometry_msgs/Twist`)

        Receives velocity command as a Twist message. The plugin will only process the linear velocity on the x and y-axis and the angular velocity on the z-axis

* **Subscribed topic**
    * `/odom` (`nav_msgs/Odometry`)   
        Nexus Robot Posiion
    
    *  `/base_link_contact_sensor_state` (`gazebo_msgs/ContactsState`)   
        Contact Sensor Data 
    
    *  `/gazebo/set_model_state` (`gazebo_msgs/ModelState`)   
        Box Position Information

    *  `/laser/scan` (`sensor_msgs/LaserScan`)   
        Laser Sensor Data : 720 × 1







***
## Environment _v1






|                                  |                                                |
| :------------------------------: | :--------------------------------------------: |
|  `nexus0 robot + lasor sensor`   |                   `video`                      |
| ![robot](/imgs/robot.png)        |          ![envv1](./imgs/env_v2.gif)           |