# Intelligent_Engineering-_SLAM
我又创造了一份垃圾qaq_SLAM
# 操作文档

## 仿真平台

`Ubuntu 16.04+ROS+Gazebo+MoveBase`

## 操作步骤

- 将*ie_car_gazebo,ie_car_nav,ie_car_teleop*三个文件夹拷贝至工作空间src目录下

- 在工作空间目录下执行

  ```xml-dtd
  $ catkin_make
  ```

- **自主建图**

  新建终端,执行

  ```xml-dtd
  $ roslaunch ie_car_gazebo ie_nav_gazebo.launch
  ```

  新建终端,执行

  ```xml-dtd
  $ roslaunch ie_car_nav exploring_slam.launch
  ```

  新建终端,执行

  ```xml-dtd
  $ rosrun ie_car_nav exploring_slam.py
  ```

  机器人开始自主建图

  建图完成后,新建终端执行

  ```xml-dtd
  $ rosrun map_server map_saver -f name_of_map
  ```

  保存地图.

- **地图标注**

  在*/catkin_ws/src/ie_car_nav/src/sweep_floor.py*文件中进行地图标注

- **扫地**

  新建终端,执行

  ```xml-dtd
  $ roslaunch ie_car_gazebo ie_nav_gazebo.launch
  ```

  新建终端,执行

  ```xml-dtd
  $ roslaunch ie_car_nav nav_with_map.launch
  ```

  新建终端,执行

  ```xml-dtd
  $ rosrun ie_car_nav sweep_floor.py
  ```

  选择指定区域让机器人扫地.

