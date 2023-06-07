# Robotics 2조팀플


## Usage


```bash
source catki_ws/devel/setup.bash
```


### make world
```bash
roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch
```


### box random moving
```bash
rosrun random_box_mover random_box_mover
```

### Agent Node
```bash
rosrun agent_node agent.py
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
