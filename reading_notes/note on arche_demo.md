# arche_demo.launch 笔记

Github地址 https://github.com/ethz-asl/voxgraph



```bash
# 运行
roslaunch voxgraph arche_demo.launch rosbag_path:=${HOME}/catkin_ws/rosbag/arche_flight1_2ms_indoor-outdoor-figure-8.bag
```



## 运行时的bug

运行时出现一下问题:

`[ WARN] [1638089247.645120985, 1562071213.553981915]: Lookup would require extrapolation into the future.  Requested time 1562071203.743470848 but the latest data is at time 1562071186.794263808, when looking up transform from frame [os1_lidar] to frame [odom]`

issue区有人提出类似问题https://github.com/ethz-asl/voxgraph/issues/58, 但并没有解决方案.

- 将lidar msg的queue size调小后上述warning不再出现
- 具体细节:
  - 在`$(find lidar_undistortion)/lidar_undistortion/src/lidar_undistortion.cpp`中, 将图中两处的100改为10.
  - ![lidar_undistorter-ln16](./pic_voxgraph/lidar_undistorter-ln16.png)

**\*TODO\***但运行一段时间后会卡死



# 程序运行流程

`arche_demo.launch`运行了两个launch文件, 

- 处理lidar的失真 [os1_undistortion.launch](#os1_undistortion)
- `$(find lidar_undistortion)/launch/os1_undistortion.launch`
- voxgraph地图(主体程序) [voxgraph_mapper.launch](#voxgraph_mapper)
- `$(find voxgraph)/launch/voxgraph_mapper.launch`





## lidar_undistortion os1_undistortion.launch

<a name=os1_undistortion></a>

- 文件中将"pointcloud" remap到 "/penguin/os1_cloud_node/points"
  - 代码中用`pointcloud`代指点云的topic, launch文件中可以用具体的topic remap到`pointcloud`来让程序正确读取
- odom_frame_id是`odom`, lidar_frame_id是`os1_lidar`
- 运行的node在`$(find lidar_undistortion)/lidar_undistortion/src/lidar_undistortion.cpp`

### lidar_undistortion.cpp

参考阅读: https://zhuanlan.zhihu.com/p/413195019

- ![lidar_undistorter-ln16](./pic_voxgraph/lidar_undistorter-ln16.png)

- 从topic `pointcloud`读取msg, 调用回调函数[`pointcloudCallback()`](#pointcloudCallback)
  - 读取的是原始点云
- 正确点云的pub topic是`pointcloud_corrected`

#### pointcloudCallback()

<a name=pointcloudCallback></a>

- ![lidar_undistorter-ln38](./pic_voxgraph/lidar_undistorter-ln38.png)
- 这次点云扫描的起始时间和结束时间
  - *扫描需要时间, 而车辆在一次扫描期间发生了移动, 所以需要知道扫描开始和结束的时间* 
- *头文件中解释说, 点云校正时, 每个点的记录的坐标系不相同, 代码中叫这个时刻S_correct. 需要把他们统一转换到一个坐标系下, 也就是这次扫描起始时刻的坐标系, 叫做scan frame(S_original). 可是我们并不能直接得到变换矩阵, 但我们有世界坐标系fixed frame(F)和S_correct的变换矩阵, 以及fixed frame(F) 和S_original的变换矩阵*
  - *某个固定的参考系叫 fixed frame, 比如世界坐标系*
  - *某次扫描的起始参考系叫S_original*
  - *该次扫描中 扫到某点时的参考系叫S_correct*
- ![lidar_undistorter-ln54](./pic_voxgraph/lidar_undistorter-ln54.png)
- T_F_S_original: fixed frame和S_original的变换矩阵
- ![lidar_undistorter-ln67](./pic_voxgraph/lidar_undistorter-ln67.png)
- 根据点的记录时间有没有变来判断是否需要更新变换矩阵
  - 根据时间确定F和S_correct的变换矩阵
  - 叠加F和S_original的变换矩阵， 得到S_original到S_correct的
- 把点变换到S_original坐标系下
- 生成msg, 并发布













## voxgraph voxgraph_mapper.launch

<a name=voxgraph_mapper></a>

- 运行node `voxgraph_mapper`, 位于`$(find voxgraph)/voxgraph/src/frontend/voxgraph_mapper.cpp`
  - 使用参数文件`$(find voxgraph)/voxgraph/config/voxgraph_mapper.yaml`

- 还有rviz的node(略)

### voxgraph_mapper.cpp

- 从ros拿参数`getParametersFromRos()`
  - 略
  - 如果`update_mesh_every_n_sec > 0`, 定时回调[`publishActiveSubmapMeshCallback()` ](#publishActiveSubmapMeshCallback)
- sub topic subscribeToTopics()`

#### `pointcloudCallback()`

- ![voxgraph_mapper-ln205](./pic_voxgraph/voxgraph_mapper-ln205.png)
- `current_timestamp`: 点云msg的时间戳
- 根据`current_timestamp`**更新`map_tracker_`**
  - **\*TODO\*** 补上详细解释
  - 更新`T_O_B_`, baselink到odom的transform
  - 更新`T_S_B_`, baselink到submap的transform
  - 更新`T_B_C_`, baselink到pointcloud的transform
- 根据`current_timestamp`判断是否需要**生成新的submap**
  - `shouldCreateNewSubmap()`判断时间是否超出时间间隔(20)
  - *注意, 生成新的子地图前需要确认优化完成*
    - *生成新图会重置约束条件, 如果有未完成的优化会导致程序崩溃*
  - ![voxgraph_mapper-ln230](./pic_voxgraph/voxgraph_mapper-ln230.png)
  - 生成新的子地图`switchToNewSubmap()`
    -  
  - 发布当前子地图mesh`publishActiveSubmapMeshCallback()`<a name=publishActiveSubmapMeshCallback></a>
    -  
  - 回调优化位姿图`optimizePoseGraph()`
    -  
  - 发布地图`publishMaps()`
    -  
  - 

#### ``loopClosureCallback()`

- 发布topic advertiseTopics()`
  - 略
- 发布service `advertiseServices()`
  - `publishSeparatedMeshCallback()`
  - `publishCombinedMeshCallback()`
  - `optimizeGraphCallback()`
  - `finishMapCallback()`
  - `saveToFileCallback()`
  - `savePoseHistoryToFileCallback()`
  - `saveSeparatedMeshCallback()`
  - `saveCombinedMeshCallback()`
  - `saveOptimizationTimesCallback()`
- 

