# vehicle detection in ros

## description
1. 在 vehicle_description 中建立车辆模型
2. 在 scan_gazebo_simu 中建立 gazebo 仿真环境
3. 在 get_scan_data 中建立激光扫描仪模型和获取仿真数据
4. 在 lds_controler 中控制激光扫描仪运动
5. 在 vehicle_detection 中进行算法处理
6. 在 pcd_data 中存储仿真点云文件


## running
### 导入文件
1. `mkdir -p ~/Documents/ros/vehicle_detection/src`
2. `cd ~/Documents/ros/vehicle_detection/src`
3. `git clone git@github.com:chunxiaoshu/vehicle_detection.git`
4. `cd ~/Documents/ros/vehicle_detection`
5. `catkin_make`
6. `source devel/setup.bash`

### 运行
1. `roscore`
2. `roslaunch scan_gazebo_simu scan_truck_rail_slant.launch`
3. `rosrun get_scan_data get_lds_data_rail_slant`
4. `rosrun lds_controler lds_rail_control`

### 其他
1. 设置是否打开gazebo界面：在运行第二条命令的时候加入`--gui=false`
