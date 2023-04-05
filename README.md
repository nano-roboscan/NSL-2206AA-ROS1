# NSL-2206AA ROS1
--- NSL-2206AA ROS1 demo ---

1. Build env
 - Ubuntu18.04
 - ROS1 Melodic
 - OPENCV 4.5.4
 
 
2. Build NSL-2206AA demo
```
$ cd NSL2206_driver
$ catkin_make
$ source ./devel/setup.bash
```
 
3. Start commands
```
$ rosrun roboscan_nsl2206 roboscan_publish_node
$ roslaunch roboscan_nsl2206 camera.Launch
```

# NSL-2206AA View

![rviz](https://user-images.githubusercontent.com/106071093/230008512-38708b55-997e-4b72-89c4-a771e6d54154.png)



# Set parameters
```
$ rqt
 (reconfigure)
```

![rqt](https://user-images.githubusercontent.com/106071093/230008611-651e479b-a96c-451e-b929-d34aeeb9be2c.png)



```

//
```

 



 
 
 
