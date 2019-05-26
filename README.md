# vehicle detection in ros

## description
车辆从-10位置处沿着x轴正方向运行，LDS在车辆上方 10m 处

在vehicle_description中建立车辆模型
在vehicle_detection中建立仿真环境
在lds_data中读取数据并处理

1. 设置是否打开gazebo界面
    * 在 demo.launch 文件中设置值为 false 不启用 rviz
    * `<arg name="gui" default="false"/>`

2. 设置是否启用rviz可视化
    * 在 demo.launch 文件中注释下面语句不启用 rviz
    * `<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />`

3. 目前没有设置车辆方向变化


## running

1. `mkdir -p ~/Documents/ros/vehicle_detection/src`
2. `cd ~/Documents/ros/vehicle_detection/src`
3. `git clone git@github.com:chunxiaoshu/vehicle_detection.git`
4. `cd ~/Documents/ros/vehicle_detection`
5. `catkin_make`
6. `source devel/setup.bash`
7. `roslaunch vehicle_detection demo.launch`


