# DAIB 双机协同实现说明

## 当前代码已经实现

1. 每架无人机独立运行 FAST-LIVO2、Explorer、Decision 和 EGO-Planner；所有本机安全链均使用独立命名空间。
2. `CoExploreTask.msg` 以 `(robot_id, session, sequence, generation)` 标识任务，并以 `valid_until` 构成短时目标租约。
3. 两机目标进入同一排斥半径时，先比较 Explorer 原始目标得分；分数近似相等时由较小 `robot_id` 获得租约。失败方记录冷却并重新选点。
4. Explorer 同时接收本机和同伴 PVBSM。查询远程覆盖前，通过 TF 将本机候选变换到远程 PVBSM 坐标；多来源结果按“任一来源已覆盖”融合，用于抑制重复探索。
5. 同伴任务心跳转换为 `SOURCE_PEER` 健康状态。TTL 超时后，Decision 收到 `PEER_LINK_LOST`，本机定位、占据更新和局部规划不会停止。
6. EGO 的广播 B 样条增加畸形、乱序和起始时间检查，并通过静态 `peer_to_local` 锚点转换远程控制点。碰撞检查按相同未来时刻比较两条轨迹，过期轨迹不再无限外推。
7. FAST-LIVO2 的 DAIB 父/子坐标名已参数化；双机入口关闭旧的同名 TF 广播，避免两套 `camera_init→aft_mapped` 在共享 ROS master 上互相覆盖。

## 当前实现边界

- 本版使用同一 ROS master（或可透明转发 ROS 话题的局域网）通信；尚未实现独立 UDP 包头、CRC、分片和缺包重传。
- 跨机坐标由静态 TF/起飞标定给出；尚未实现基于公共子地图的自动在线对齐和错误对齐撤销。
- 当前租约粒度是“已选目标及空间排斥半径”，不是完整 Frontier 簇拍卖；已能避免两机同时前往同一区域，但负载均衡仍可扩展。
- 同伴失联后租约会自然过期，本机继续本地探索；“指定未完成区域接管”和持久化任务账本仍需后续实现。
- EGO 同伴轨迹仍使用上游 `Bspline.msg`，session/generation 由任务层保证；轨迹消息本身尚未扩展会话字段。

## 启动与坐标约定

1. 两个 FAST-LIVO2 实例分别使用
   `mapping_mid70_d435i_daib_uav.launch`，设置不同 `uav_name`、`robot_id` 和传感器话题。
2. DAIB 输出坐标必须分别为 `uav0/camera_init` 与
   `uav1/camera_init`。
   两套 `mapping_mid70_d435i_daib_uav.launch` 默认关闭 legacy TF；不要再由两个进程广播同名父子坐标。
3. 使用 `dual_uav_cooperation.launch` 启动两套 Explorer、Decision 和 Planner，并填写 `uav1_in_uav0_*` 及其逆变换。
4. 真机使用同一时钟源；任务租约和 B 样条接收都会拒绝过期数据。

示例（传感器话题按实际驱动替换）：

```bash
roslaunch fast_livo mapping_mid70_d435i_daib_uav.launch \
  uav_name:=uav0 robot_id:=0 lidar_topic:=/uav0/livox/lidar \
  imu_topic:=/uav0/livox/imu image_topic:=/uav0/camera/color/image_raw
roslaunch fast_livo mapping_mid70_d435i_daib_uav.launch \
  uav_name:=uav1 robot_id:=1 lidar_topic:=/uav1/livox/lidar \
  imu_topic:=/uav1/livox/imu image_topic:=/uav1/camera/color/image_raw
roslaunch daib_explorer dual_uav_cooperation.launch \
  uav1_in_uav0_x:=0.0 uav1_in_uav0_y:=0.0 uav1_in_uav0_z:=0.0 \
  uav1_in_uav0_yaw_rad:=0.0
```

非零锚点时，还必须同步填写 EGO 使用的角度制变换及其逆变换；错误的逆变换会直接导致同伴轨迹位置错误。

## 最小验证顺序

1. 只启两套 Explorer，确认 `/daib_coexplore/task` 中 robot_id 0/1 均以 1 Hz 更新。
2. 给两机相同 Frontier，确认最多一机保留该目标，另一机日志中 `peer_goal` 拒绝数增加并发布新 Generation。
3. 回放两路 PVBSM，确认每个 Explorer 的 memory stats 最后一个字段为 `source_count=2`，且远程已覆盖区域降低候选得分。
4. 断开共享话题 3 s，确认 Decision 收到 `PEER_LINK_LOST`，但本机 `/daib_slam/odom` 和规划云仍连续。
5. 发布相交 B 样条，确认 EGO 在安全距离内触发重规划；再发送乱序、畸形或过期轨迹，确认其被拒绝。
