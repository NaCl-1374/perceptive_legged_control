# perceptive_legged_control 项目编译和仿真指南
## 项目介绍
主要实现了 **Perceptive Locomotion Through Nonlinear Model-Predictive Control**
测试环境为Vmware虚拟机 运行Ubuntu 20.04 使用I5 12490F 4.5Ghz 分配给虚拟机2个核心四个线程。
参考廖佬的**legged_control**的仿真框架，进行测试。
实际运行时，在梅花桩的地形成功率较低。求解很容易崩溃。
在虚拟机仿真中，需要使用预先制作的高程图，若使用cpu版本的高程图，在大幅机动时，感知运算处理较慢，会导致建图出错，进而导致运动规划出错。在实体机运行时，可以使用gpu版本的高程图。传感器使用深度相机，经过低通滤波和自碰撞滤波。
本项目使用 **catkin tools** 进行编译，包含 **debug** 和 **release** 配置。如遇到任何报错，请自行排查并调整相应的路径。
该仿真视频 [Bilibili 视频链接](https://www.bilibili.com/video/BV1hM4m1D7BM/)。
## 编译

### Catkin Tools 配置
如果需要编译 **debug** 版本，需要切换 **catkin tools** 的配置文件至 debug 配置。**release** 版本的编译文件夹后缀为 `_rel`。

### 步骤 1：编译 OCS2_ws
进入 `OCS2_ws` 目录，执行以下命令：

```bash
catkin build ocs2_legged_robot ocs2_legged_robot_ros ocs2_self_collision_visualization ocs2_anymal ocs2_anymal_loopshaping_mpc
catkin build convex_plane_decomposition
catkin build grid_map
```

编译完成后，应生成以下包：

- `../OCS2_ws/build/blasfeo_catkin`
- `../OCS2_ws/build/catkin_tools_prebuild`
- `../OCS2_ws/build/cgal5_catkin`
- `../OCS2_ws/build/convex_plane_decomposition`
- `../OCS2_ws/build/convex_plane_decomposition_msgs`
- `../OCS2_ws/build/convex_plane_decomposition_ros`
- `../OCS2_ws/build/grid_map_core`
- `../OCS2_ws/build/grid_map_cv`
- `../OCS2_ws/build/grid_map_demos`
- `../OCS2_ws/build/grid_map_filters`
- `../OCS2_ws/build/grid_map_filters_rsl`
- `../OCS2_ws/build/grid_map_loader`
- `../OCS2_ws/build/grid_map_msgs`
- `../OCS2_ws/build/grid_map_octomap`
- `../OCS2_ws/build/grid_map_ros`
- `../OCS2_ws/build/grid_map_rviz_plugin`
- `../OCS2_ws/build/grid_map_sdf`
- `../OCS2_ws/build/grid_map_visualization`
- `../OCS2_ws/build/hpipm_catkin`
- `../OCS2_ws/build/hpp-fcl`
- `../OCS2_ws/build/ocs2_anymal_commands`
- `../OCS2_ws/build/ocs2_anymal_loopshaping_mpc`
- `../OCS2_ws/build/ocs2_anymal_models`
- `../OCS2_ws/build/ocs2_anymal_mpc`
- `../OCS2_ws/build/ocs2_centroidal_model`
- `../OCS2_ws/build/ocs2_core`
- `../OCS2_ws/build/ocs2_ddp`
- `../OCS2_ws/build/ocs2_frank_wolfe`
- `../OCS2_ws/build/ocs2_ipm`
- `../OCS2_ws/build/ocs2_legged_robot`
- `../OCS2_ws/build/ocs2_legged_robot_ros`
- `../OCS2_ws/build/ocs2_mpc`
- `../OCS2_ws/build/ocs2_msgs`
- `../OCS2_ws/build/ocs2_oc`
- `../OCS2_ws/build/ocs2_pinocchio_interface`
- `../OCS2_ws/build/ocs2_qp_solver`
- `../OCS2_ws/build/ocs2_quadruped_interface`
- `../OCS2_ws/build/ocs2_quadruped_loopshaping_interface`
- `../OCS2_ws/build/ocs2_robotic_assets`
- `../OCS2_ws/build/ocs2_robotic_tools`
- `../OCS2_ws/build/ocs2_ros_interfaces`
- `../OCS2_ws/build/ocs2_self_collision`
- `../OCS2_ws/build/ocs2_self_collision_visualization`
- `../OCS2_ws/build/ocs2_sqp`
- `../OCS2_ws/build/ocs2_switched_model_interface`
- `../OCS2_ws/build/ocs2_switched_model_msgs`
- `../OCS2_ws/build/ocs2_thirdparty`
- `../OCS2_ws/build/pinocchio`
- `../OCS2_ws/build/segmented_planes_terrain_model`

### 步骤 2：编译 perceptive_ocs2_ws 
进入 `perceptive_ocs2_ws` 目录 。由于该仿真修改自 `legged_control`，因此可以参考他的编译过程。在编译前，需修改 `.catkin_tools/profiles/default/config.yaml` 文件，将当前工作空间扩展链接至 `OCS2_ws`。

执行以下命令进行编译：

```bash
catkin build plot_terrain
catkin build perceptive_trajectories
catkin build legged_perceptive_description
catkin build legged_perceptive_description_custom
catkin build legged_unitree_description
catkin build ocs2_legged_estimation
catkin build ocs2_legged_self_collishion
catkin build ocs2_legged_wbc
catkin build ocs2_unitree_controllers
catkin build ocs2_unitree_loopshaping_mpc
catkin build ocs2_unitree_models
catkin build ocs2_unitree_mpc
```

编译后应生成以下包：

- `build_rel/catkin_tools_prebuild`
- `build_rel/legged_common`
- `build_rel/legged_gazebo`
- `build_rel/legged_perceptive_description`
- `build_rel/legged_perceptive_description_custom`
- `build_rel/legged_unitree_description`
- `build_rel/ocs2_legged_estimation`
- `build_rel/ocs2_legged_self_collishion`
- `build_rel/ocs2_legged_wbc`
- `build_rel/ocs2_unitree_controllers`
- `build_rel/ocs2_unitree_loopshaping_mpc`
- `build_rel/ocs2_unitree_models`
- `build_rel/ocs2_unitree_mpc`
- `build_rel/perceptive_trajectories`
- `build_rel/plot_terrain`
- `build_rel/qpoases_catkin`

## 启动仿真

### Gazebo 仿真
仿真启动顺序必须严格遵循，因为仿真的定位数据来源于控制器，因此需要先启动控制器，然后启动地图发布器。地图发布直接使用地图的真值数据。

1. 加载 Gazebo 地形图时，会在主目录生成 `/home/me/.gazebo/models/stepping_stones_terrain` 文件夹，该文件夹存储地形数据。

2. 启动仿真：
```bash
roslaunch legged_perceptive_description_custom empty_world.launch
```

3. 启动控制器：
```bash
roslaunch ocs2_unitree_controllers unitree.launch
```

4. 启动地图发布器：
```bash
roslaunch plot_terrain convex.launch
```

5. 启动键盘控制：
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Rviz 仿真
该项目支持使用 **Rviz** 的 `nav` 目标点功能，具体实现在 `src/perceptive_trajectories` 中。

### Dummy 仿真
```bash
roslaunch ocs2_unitree_mpc unitree.launch
```

### 发布地图
```bash
roslaunch plot_terrain convex.launch
```

## 注意事项

1. 如果更改了 Gazebo 地形图，需要删除 `/home/me/.gazebo/paging` 文件夹，这样 Gazebo 地形才能更新。

2. `plot_terrain` 包使用的地形图与 Gazebo 存在 90 度的偏差，需要注意。

3. 如果要更改地形图，可以使用修图工具对图片进行修改，生成旋转 90 度后的图片，分别放到 `perceptive_ocs2_ws/src/plot_terrain/data` 目录和 `/home/me/.gazebo/models/stepping_stones_terrain` 目录，并修改对应的 launch 文件。

4. 运行前需要将地图放到对应的目录下`/home/me/.gazebo/models/stepping_stones_terrain` 
