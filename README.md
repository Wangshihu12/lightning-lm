# Lightning-LM

Lightning-Speed Lidar Localization and Mapping

Lightning-LM is a complete laser mapping and localization module.

Features of Lightning-LM:

1. [done] Complete 3D Lidar SLAM, fast LIO front-end (AA-FasterLIO), standard
2. [done] 3D to 2D map conversion (g2p5), optional, if selected outputs real-time 2D grid map, can be saved
3. [done] Real-time loop closure detection, standard, performs back-end loop closure detection and correction if
   selected
4. [done] Smooth high-precision 3D Lidar localization, standard
5. [done] Dynamic loading scheme for map partitions, suitable for large-scale scenes
6. [done] Localization with separate dynamic and static layers, adaptable to dynamic scenes, selectable strategies for
   dynamic layer, optional, if selected saves dynamic layer map content, three strategies available (short-term,
   medium-term, permanent), default is permanent
7. [done] High-frequency IMU smooth output, standard, 100Hz
8. GPS geoinformation association, optional (TODO)
9. Vehicle odometry input, optional (TODO)
10. [done] Lightweight optimization library miao and incremental optimization (derived from g2o, but lighter and faster,
    supports incremental optimization, no need to rebuild optimization model), standard, used in both loop closure and
    localization
11. [done] Two verification schemes: offline and online. Offline allows breakpoint debugging with strong consistency.
    Online allows multi-threaded concurrency, fast processing speed, dynamic frame skipping, and low resource usage.
12. [done] High-frequency output based on extrapolator and smoother, adjustable smoothing factor
13. [done] High-performance computing: All the above features can run using less than one CPU core on the pure CPU
    side (online localization 0.8 cores, mapping 1.2 cores, 32-line LiDAR, without UI).

## Updates

1. 2025.11.13

- Fix two logic typos in FasterLIO.
- Add global height constraint that can be configured in loop_closing.with_height. If set true, lightning-lm will keep
  the output map into the same height to avoid the Z-axis drifting in large scenes. It should not be set if you are
  using lightning-lm in scenes that have multi-floor or stairs structures.

## Examples

- Mapping on the VBR campus dataset:

<video controls width="100%">
  <source src="doc/slam_vbr.mp4" type="video/mp4">
</video>

- Localization on VBR

<video controls width="100%">
  <source src="doc/lm_loc_vbr_campus.mp4" type="video/mp4">
</video>

- Map on VBR
    - Point Cloud

  ![](./doc/campus_vbr.png)
    - Grid Map

  ![](./doc/campus.png)

- Localization on the NCLT dataset

<video controls width="100%">
  <source src="doc/lm_loc1_nclt.mp4" type="video/mp4">
</video>

## Build

### Environment

- Ubuntu 20.04 with ROS Noetic (desktop-full) is the primary target.
- Ubuntu 22.04 can also be used if Noetic is installed via Docker or third-party packages.

### Dependencies

- ROS Noetic (roscpp, tf2_ros, tf2, rosbag, pcl_ros, pcl_conversions, catkin)
- Pangolin (for visualization, see thirdparty)
- OpenCV
- PCL
- yaml-cpp
- glog
- gflags

On Ubuntu 20.04/Noetic you can install the system packages with:

```bash
./scripts/install_dep.sh
```

### Build

Use a standard catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your_org/lightning-lm.git
cd ..
catkin_make    # or: catkin build
source devel/setup.bash
```

### Build Results

After building, you will get the corresponding online/offline mapping and localization programs for this package. The
offline programs are suitable for scenarios with offline data packets to quickly obtain mapping/localization results,
while the online programs are suitable for scenarios with actual sensors to obtain real-time results.

For example, calling the offline mapping program on the NCLT dataset:

```bash
rosrun lightning run_slam_offline --input_bag ~/data/NCLT/20130110/20130110.bag --config ./config/default_nclt.yaml
```

For the online version, replace `run_slam_offline` with `run_slam_online`.

## Testing on Datasets

You can directly use our converted ROS1 `.bag` datasets. If you download the original logs, please convert them into
ROS1 bag files (`.bag`) following your preferred pipeline (e.g., replay the dataset with a driver and use `rosbag record`).

Converted dataset addresses:

- OneDrive: https://1drv.ms/f/c/1a7361d22c554503/EpDSys0bWbxDhNGDYL_O0hUBa2OnhNRvNo2Gey2id7QMQA?e=7Ui0f5
- BaiduYun:

Original dataset addresses:

- NCLT dataset: http://robots.engin.umich.edu/nclt/
- UrbanLoco dataset: https://github.com/weisongwen/UrbanLoco
- VBR dataset: https://www.rvp-group.net/slam-dataset.html

### Mapping Test

1. Real-time mapping (real-time bag playback)
    - Start the mapping program:
      ```bash
      rosrun lightning run_slam_online --config ./config/default_nclt.yaml
      ```
    - Play the data bag with `rosbag play your_data.bag`
    - Save the map
      ```bash
      rosservice call /lightning/save_map "map_id: 'new_map'"
      ```
2. Offline mapping (traverse data, faster)
    - ```bash
      rosrun lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag [bag_file]
      ```
    - The map is automatically saved to `data/new_map` after finishing.
3. Viewing the map
    - View the full map: ```pcl_viewer ./data/new_map/global.pcd```
    - The actual map is stored in blocks, global.pcd is only for displaying the result.
    - map.pgm stores the 2D grid map information.
    - Note that during the localization program run or upon exit, results for dynamic layers might also be stored in the
      same directory, so there might be more files.

### Localization Test

1. Real-time localization
    - Write the map path to `system.map_path` in the yaml file, default is `new_map` (consistent with the mapping
      default).
    - Place the vehicle at the mapping starting point.
    - Start the localization program:
      ```bash
      rosrun lightning run_loc_online --config ./config/default_nclt.yaml
      ```
    - Play the bag or stream live sensor data.
2. Offline localization
    - ```bash
      rosrun lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag [bag_file]
      ```
3. Receiving localization results
    - The localization program outputs TF topics at the same frequency as the IMU (50-100Hz).

### Debugging on Your Own Device

First, you need to know your LiDAR type and set the corresponding `fasterlio.lidar_type`. Set it to 1 for Livox series,
2 for Velodyne, 3 for Ouster.
If it's not one of the above types, you can refer to the Velodyne setup method.

A simpler way is to first record a ROS1 bag, get offline mapping and localization working, and then debug the online
situation.

You usually need to modify `common.lidar_topic` and `common.imu_topic` to set the LiDAR and IMU topics.

The IMU and LiDAR extrinsic parameters can default to zero; we are not sensitive to them.

The `fasterlio.time_scale` related to timestamps is sensitive. You should pay attention to whether the LiDAR point cloud
has timestamps for each point and if they are calculated correctly. This code is in `core/lio/pointcloud_preprocess`.

Refer to the next section for other parameter adjustments.

### Fine-tuning Lightning-LM

You can fine-tune Lightning by modifying the configuration file, turning some features on or off. Common configuration
items include:

- `system.with_loop_closing` Whether loop closure detection is needed
- `system.with_ui` Whether 3D UI is needed
- `system.with_2dui` Whether 2D UI is needed
- `system.with_g2p5` Whether grid map is needed
- `system.map_path` Storage path for the map
- `fasterlio.point_filter_num` Point sampling number. Increasing this results in fewer points, faster computation, but
  not recommended to set above 10.
- `g2p5.esti_floor` Whether g2p5 needs to dynamically estimate ground parameters. If the LiDAR rotates horizontally and
  the height is constant, you can turn this option off.
- `g2p5.grid_map_resolution` Resolution of the grid map

### TODO

- [done] UI displays trajectory after loop closure
- [done] Grid map saved in ROS-compatible format
- [done] Check if grid map resolution values are normal
- Force 2D output
- Additional convenience features (turn localization on/off, reinitialize, specify location, etc.)

## Miscellaneous

1. Merging ROS1 bags  
   Use `scripts/merge_bags.py` to concatenate multiple `.bag` files into a single bag:
   ```bash
   python3 scripts/merge_bags.py /path/to/bag_dir /path/to/output.bag
   ```

---

Lightning-LM 是一个完整的激光建图+定位模块。

Lightning-LM特性：

1. [done] 完整的3D Lidar SLAM，快速的LIO前端（AA-FasterLIO），标配
2. [done] 3D至2D地图转换（g2p5），选配，选上的话会输出实时的2D栅格，可以保存
3. [done] 实时回环检测，标配，选上的话会进行后端回环检测并闭环
4. [done] 流畅的高精3D Lidar 定位，标配
5. [done] 地图分区动态加载方案，适用大场景
6. [done] 动静态图层分离定位，适配动态场景，可选择动态图层的策略，选配，选上的话会保存动态图层的地图内容，有三种策略可以选（短期、中期、永久），默认永久
7. [done] 高频率IMU平滑输出，标配，100Hz
8. GPS地理信息关联，选配 (TODO)
9. 车辆里程计输入，选配 (TODO)
10. [done] 轻量优化库miao以及增量式优化（来自g2o，但更轻更快，支持增量优化，不需要重新构建优化模型），标配，在回环、定位中均有用到
11. [done] 离线与在线两种验证方案。离线可以断点调试，一致性强。在线可以多线程并发，处理速度快，可以设置动态跳帧，占用低。
12. [done] 基于外推器和平滑器的高频率输出，平滑因子可调
13. [done] 高性能计算：以上这些特性在纯CPU端不到一个核心就可以运行（在线定位0.8个核，建图1.2个核，32线雷达，无UI情况）

## 更新

### 2025.11.13

- 修复了FasterLIO中的两个逻辑问题
- 增加了高度约束，在loop_closing.with_height中配置。配置高度约束以后，lightning会保障输出地图的水平度（限制Z轴飘移），但这样就不适用于多层室内之类的带有立体结构的场景。

## 案例

- VBR campus数据集上的建图：

<video controls width="100%">
  <source src="doc/slam_vbr.mp4" type="video/mp4">
</video>

- VBR上的定位

<video controls width="100%">
  <source src="doc/lm_loc_vbr_campus.mp4" type="video/mp4"> 
</video>

- VBR上的地图
    - 点云

  ![](./doc/campus_vbr.png)
    - 栅格

  ![](./doc/campus.png)

- NCLT 数据集上的定位

<video controls width="100%">
  <source src="doc/lm_loc1_nclt.mp4" type="video/mp4"> 
</video>

## 编译

### 环境

- 推荐 Ubuntu 20.04 + ROS Noetic（desktop-full）。
- 如果使用 Ubuntu 22.04，可以通过 Docker 或第三方源安装 Noetic。

### 依赖

- ROS Noetic（含 roscpp / tf2_ros / rosbag / pcl_ros / pcl_conversions / catkin）
- Pangolin（用于可视化，见 thirdparty）
- OpenCV
- PCL
- yaml-cpp
- glog
- gflags

在 Ubuntu 20.04/Noetic 上可直接运行：

```bash
./scripts/install_dep.sh
```

### 编译

在 catkin 工作空间中编译：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your_org/lightning-lm.git
cd ..
catkin_make         # 或 catkin build
source devel/setup.bash
```

### 编译结果

编译后会得到对应的在线/离线建图和定位程序。离线程序适合有离线 bag，想快速得到建图/定位结果的场景；在线程序适合实时传感器输入。

例如：在 NCLT 数据集上调用离线建图程序：

```bash
rosrun lightning run_slam_offline --input_bag ~/data/NCLT/20130110/20130110.bag --config ./config/default_nclt.yaml
```

若需要在线版本，只需把 `run_slam_offline` 替换成 `run_slam_online`。

## 在数据集上测试

您可以直接使用我们转换好的 ROS1 `.bag` 数据集。如果需要原始数据集，可自行通过驱动回放并使用 `rosbag record` 转换为 `.bag`。

转换后的数据集地址：

- OneDrive: https://1drv.ms/f/c/1a7361d22c554503/EpDSys0bWbxDhNGDYL_O0hUBa2OnhNRvNo2Gey2id7QMQA?e=7Ui0f5
- BaiduYun:

原始数据集地址：

- NCLT 数据集：http://robots.engin.umich.edu/nclt/
- UrbanLoco 数据集： https://github.com/weisongwen/UrbanLoco
- VBR 数据集：https://www.rvp-group.net/slam-dataset.html

### 建图测试

1. 实时建图（实时播包）
    - 启动建图程序:
      ```bash
      rosrun lightning run_slam_online --config ./config/default_nclt.yaml
      ```
    - 用 `rosbag play your_data.bag` 播放数据包
    - 保存地图
      ```bash
      rosservice call /lightning/save_map "map_id: 'new_map'"
      ```
2. 离线建图（遍历跑数据，更快一些）
    - ```bash
      rosrun lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag 数据包
      ```
    - 结束后会自动保存至 `data/new_map` 目录下
3. 查看地图
    - 查看完整地图：```pcl_viewer ./data/new_map/global.pcd```
    - 实际地图是分块存储的，global.pcd仅用于显示结果
    - map.pgm存储了2D栅格地图信息
    - 请注意，在定位程序运行过程中或退出时，也可能在同目录存储动态图层的结果，所以文件可能会有更多。

### 定位测试

1. 实时定位
    - 将地图路径写到yaml中的 system-map_path 下，默认是new_map（和建图默认一致)
    - 将车放在建图起点处
    - 启动定位程序：
      ```bash
      rosrun lightning run_loc_online --config ./config/default_nclt.yaml
      ```
    - 播包或者输入实时传感器数据即可

2. 离线定位
    - ```bash
      rosrun lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag 数据包
      ```

3. 接收定位结果
    - 定位程序输出与IMU同频的TF话题（50-100Hz）

### 在您自己的设备上调试

首先您需要知道自己的雷达类型，设置对应的fasterlio.lidar_type类型。livox系列的配置成1，Velodyne的设置成2,ouster设置成3.
如果不在以上种类，可以参考velodyne的设置方式。

比较简单的方式是先录一个 ROS1 数据包，将离线建图、定位调通后，再去调试在线的情况。

您通常需要修改common.lidar_topic和common.imu_topic来设置雷达与imu的话题。

imu和雷达外参默认为零就好，我们对这个不敏感。

时间戳相关的fasterlio.time_scale是敏感的。您最好关注一下雷达点云是否带有每个点的时间戳，以及它们是否计算正确。这些代码在core/lio/pointcloud_preprocess里.

其他参数调整参考下一节。

### 对lightning-lm进行微调

您可以在配置文件中对lightning进行微调，打开或者关闭一些功能。常见的配置项有：

- system.with_loop_closing 是否需要回环检测
- system.with_ui 是否需要3DUI
- system.with_2dui 是否需要2DUI
- system.with_g2p5 是否需要栅格地图
- system.map_path 地图的存储路径
- fasterlio.point_filter_num 点的采样数。调大后点数会少一些，计算更快，但不建议调到10以上。
- g2p5.esti_floor g2p5是否需要动态估计地面参数。如果雷达水平旋转且高度不变，可以关闭此选项.
- g2p5.grid_map_resolution 栅格地图的分辨率

### TODO

- [done] UI显示闭环后轨迹
- [done] 栅格地图保存为兼容ROS的形式
- [done] 检查栅格地图的分辨率取值是否正常
- 强制2D输出
- 额外便利性功能（打开关闭定位，重新初始化，指定位置等）

## 其他

1. 合并 ROS1 bag  
   使用 `scripts/merge_bags.py` 可以把多个 `.bag` 合并为一个：
   ```bash
   python3 scripts/merge_bags.py /path/to/bag_dir /path/to/output.bag
   ```
