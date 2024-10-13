# ros_reuleaux_ws
a simple demo for use ros_reuleaux_ws to show robot worksapce | IK Fast 

# how to build
```bash
# 编译使用
catkin build map_creator biped_s4 workspace_visualization

# 生成dae文件
rosrun collada_urdf urdf_to_collada biped_s4_left_arm.urdf biped_s4_left_arm.dae
rosrun moveit_kinematics round_collada_numbers.py biped_s4_left_arm.dae biped_s4_left_arm.dae 5

# ik-fast 文件生成 | 借助已有的镜像
## 进入镜像
xhost + && sudo docker run  -it --rm  -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e DISPLAY=unix$DISPLAY  -v `pwd`:`pwd`  -w `pwd` fishros2/openrave

## 先查看joint和link的信息
openrave-robot.py biped_s4_left_arm.dae --info links
openrave-robot.py biped_s4_left_arm.dae --info joints

## 生成
### 1 9 8 # 失败
### 1 9 7 # 失败
#python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=9 --freeindex=7 --savefile=$(pwd)/ikfastBiped2.cpp
#python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=9 --freeindex=8 --savefile=$(pwd)/ikfastBiped2.cpp

### 1 9 6 测试中 | 成功有效果
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=9 --freeindex=6 --savefile=$(pwd)/ikfastBiped.cpp

### 1 8 6 测试中 | 感觉末端ik的结果不是很对
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=biped_s4_left_arm.dae --iktype=transform6d --baselink=1 --eelink=8 --freeindex=6 --savefile=$(pwd)/ikfastBiped3.cpp

# 有了ikfast，开始生成可达图
rosrun map_creator create_reachability_map 0.05
rosrun map_creator create_reachability_map 0.08

rosrun map_creator create_reachability_map 0.05 biped_s4_186_0.05.h5 # 给一个特殊名称
rosrun map_creator create_reachability_map 0.08 biped_s4_186_0.08.h5 # 给一个特殊名称

# 查看可达图
h5dump /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_186_0.08_2.h5

# 加载/发布 可达图
rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_r0.05_reachability.h5
rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_r0.08_reachability.h5

rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_186_0.05.h5
rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_186_0.08.h5
rosrun map_creator load_reachability_map /home/lab/ros_reuleaux_ws/src/map_creator/maps/biped_s4_186_0.08_2.h5
```
```bash
root@9c2c477c1ff8:/home/lab/ros_reuleaux_ws/src/biped_s4/urdf# openrave-robot.py biped_s4_left_arm.dae --info links
name               index parents        
----------------------------------------
world_link         0                    
base_link          1     world_link     
l_arm_pitch        2     base_link      
l_arm_roll         3     l_arm_pitch    
l_arm_yaw          4     l_arm_roll     
l_forearm_pitch    5     l_arm_yaw      
l_hand_yaw         6     l_forearm_pitch
l_hand_pitch       7     l_hand_yaw     
l_hand_roll        8     l_hand_pitch   
l_hand_end_virtual 9     l_hand_roll    
----------------------------------------
name               index parents  
```