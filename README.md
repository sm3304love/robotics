# Robotics 2조팀플


## Usage




**every Terminal**
```bash
source catkin_ws/devel/setup.bash
```


#### launch World
```bash
roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch
```

#### box random moving
```bash
rosrun random_box_mover random_box_mover
```

#### Environment Node + Conflict Regenerate
```bash
rosrun agent_node running.py
```










***
## Agent Node


#### Subscribed Topics


* **Published Topics**
    * `/cmd_vel` (`geometry_msgs/Twist`)

        Receives velocity command as a Twist message. The plugin will only process the linear velocity on the x and y-axis and the angular velocity on the z-axis

* **Subscribed topic**
    * `/gazebo/model_states` (`gazebo_msgs/ModelStates`)   
        Nexus Robot Posiion
    
    *  `/base_link_contact_sensor_state` (`gazebo_msgs/ContactsState`)   
        Contact Sensor Data 
    
    *  `/gazebo/set_model_state` (`gazebo_msgs/ModelState`)   
        Obstacle & Target Pillar Position

    *  `/laser/scan` (`sensor_msgs/LaserScan`)   
        Laser Sensor Data : 720 × 1






###
* Observation   

**State**
```python
def get_state(self):
    """
    State : Laser, Robot Position, Target Box Position
    """
    state = np.concatenate((self.laser_observation, 
                            [self.nexus_x, self.nexus_y, self.nexus_rz], 
                            self.Box_dict['box_target']), axis=0)
    return state
```

**Reward**
```python
def get_reward(self):
    """
    Reward Setting : Distance between Robot and Target Box
    """
    reward = 2-np.sqrt((self.nexus_x - self.Box_dict['box_target'][0])**2 + (self.nexus_y - self.Box_dict['box_target'][1])**2)
    return reward
```








***
## Environment 






|                                  |                                  |
| :------------------------------: | :------------------------------: |
|  `nexus0 robot + lasor sensor`   |                                  |
| ![robot](/imgs/robot.png)        | ![robot](/imgs/env_pic.png)      |
|                                  |                                  |
|  ![envv1](./imgs/env_v2.gif)     | ![robot](/imgs/robot_stic.png)   |