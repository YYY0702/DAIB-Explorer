# DAIB 双机部署与联调操作手册

> 适用版本：FAST-LIVO2YYY `6eaed91`、DAIB-Explorer `main（本次协同提交）`、
> DAIB-Decision `ffe6836`、ego-planner-swarmYYY `934c3b4`。
>
> 本手册面向“单机定位—探索—规划已经能够运行，现在扩展为双机”的场景。
> 首轮验证一律卸桨或使用SITL；确认坐标、时间和安全状态正确后，才连接PX4执行机构。

## 1. 双机版本增加了什么

每架无人机仍独立完成本机闭环：

```text
传感器 → FAST-LIVO2 → Explorer → Decision → EGO-Planner → PX4适配器
```

双机之间只共享三类低频或必要信息：

1. `/daib_coexplore/task`：目标租约、目标得分和Generation，用于避免两机同时前往同一区域；
2. `/daib_coexplore/pvbsm_sync`：带会话、序号、校验与NACK补发的PVBSM同步总线；
3. `/broadcast_bspline`：两机的局部B样条轨迹，用于同伴轨迹碰撞检查。

定位、局部占据地图和本机安全链不依赖对端持续在线。对端通信超过TTL后，目标租约自动过期，Decision收到`PEER_LINK_LOST`，本机继续使用自己的地图和目标运行。

## 2. 部署前先选择运行拓扑

### 2.1 首轮推荐：集中式上层联调

当前仓库已经提供的一键入口是：

```text
无人机0板：ROS Master + UAV0 FAST-LIVO2 + 两套Explorer/Decision/Planner
无人机1板：UAV1 FAST-LIVO2
```

两台板连接同一局域网。无人机1的LIVO数据传到无人机0板，双机Explorer、Decision和Planner由
`dual_uav_cooperation.launch`统一启动。

该模式最适合第一次验证，优点是启动简单、日志集中、不会重复发布协同节点；缺点是上层计算集中在一块板上，不能作为最终断链自治形态。

**注意：`dual_uav_cooperation.launch`只能启动一次。不要在两台板上各启动一次，否则会出现四套Explorer/Planner、重复节点名和重复TF。**

### 2.2 最终比赛：双板分布式运行

最终形态应为每块板各运行一套完整本机链路，仅共享任务、PVBSM和轨迹话题。现有算法接口已经支持这种数据关系，但当前仓库尚未提供可直接运行两次的单机协同编排文件。因此建议先按2.1完成整链验证，再增加参数化的`daib_vehicle.launch`，把当前双机启动文件中的`uav0`和`uav1`组分别部署到对应板卡。

在该分布式启动文件完成前，不要用复制并手工删除XML片段的方式进行真机飞行。

## 3. 网络与Docker配置

以下使用示例地址：

| 设备 | 地址 | 职责 |
|---|---|---|
| UAV0 Orange Pi | `192.168.10.10` | ROS Master及首轮上层协同节点 |
| UAV1 Orange Pi | `192.168.10.11` | 第二套传感器驱动和FAST-LIVO2 |

两机先确认互通：

```bash
ping 192.168.10.10
ping 192.168.10.11
```

Docker必须使用主机网络，否则ROS1可能向对端公布不可达的容器地址：

```bash
docker run --network host ...
```

UAV0容器中设置：

```bash
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.10
unset ROS_HOSTNAME
```

UAV1容器中设置：

```bash
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.11
unset ROS_HOSTNAME
```

建议将上述变量写入各自容器的启动脚本，而不是在两个容器中复制同一份`.bashrc`。

### 3.1 时间同步

目标租约默认TTL为3 s，轨迹还会检查起始时间和新鲜度，因此两机系统时钟必须同步。建议使用`chrony`让UAV1跟随UAV0或共同的局域网时间源，并在启动算法前检查：

```bash
chronyc tracking
chronyc sources -v
date +%s.%N
```

两机时间误差应尽量控制在50 ms以内，至少不得超过代码中的未来时间容差0.5 s。Bag回放时两机必须统一使用`/use_sim_time=true`和同一个`/clock`；真机时使用`false`。

## 4. 代码与编译

四个仓库应位于同一个catkin工作空间的`src`下，或者通过软链接加入该目录。两台板使用同一组Commit：

```bash
cd ~/daib_ws/src/FAST-LIVO2YYY
git checkout 6eaed91

cd ~/daib_ws/src/DAIB-Explorer
git checkout main
git pull --ff-only origin main

cd ~/daib_ws/src/DAIB-Decision
git checkout ffe6836

cd ~/daib_ws/src/ego-planner-swarmYYY
git checkout 934c3b4
```

在两台板上重新编译，不能只复制旧的`devel`目录，因为协同版本需要生成
`CoExploreTask.msg`和新增的`PvbsmSync.msg`：

```bash
cd ~/daib_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

检查新增消息是否生成：

```bash
rosmsg show daib_explorer/CoExploreTask
rosmsg show daib_explorer/PvbsmSync
```

若找不到消息，优先检查是否重新执行了`catkin_make`以及当前终端是否`source ~/daib_ws/devel/setup.bash`。

## 5. 话题和坐标命名

两机话题必须完全分开：

```text
/uav0/daib_slam/*
/uav0/daib_explorer/*
/uav0/daib_decision/*
/uav0/daib_ego/*

/uav1/daib_slam/*
/uav1/daib_explorer/*
/uav1/daib_decision/*
/uav1/daib_ego/*
```

共享话题主要包括：

```text
/daib_coexplore/task
/daib_coexplore/pvbsm_sync
/broadcast_bspline
```

两套局部坐标分别命名为：

```text
uav0/camera_init
uav1/camera_init
```

不要让两套FAST-LIVO2继续发布同名`camera_init → aft_mapped`。双机LIVO启动文件已默认设置
`publish_legacy_tf=false`。

## 6. 标定双机初始坐标关系

系统需要知道“UAV1局部坐标系在UAV0局部坐标系中的初始位姿”。记为：

```text
T_uav0_uav1 = (x, y, z, yaw)
```

其中`x、y、z`单位为米，`yaw`在静态TF参数中使用弧度，在EGO参数中使用角度。

如果两机在同一点、同航向启动，可全部设置为0。若不是同一点，必须测量起飞初始位置和航向，不能简单假设为0。

给定UAV1在UAV0中的位置`(x, y, z)`和航向`ψ`，其逆变换为：

```text
x_inv   = -cos(ψ)·x - sin(ψ)·y
y_inv   =  sin(ψ)·x - cos(ψ)·y
z_inv   = -z
yaw_inv = -ψ
```

例如UAV1位于UAV0坐标中的`(3.0, 1.0, 0.0)`，航向差为`30°`：

```text
uav1_in_uav0_x       = 3.0
uav1_in_uav0_y       = 1.0
uav1_in_uav0_z       = 0.0
uav1_in_uav0_yaw_rad = 0.523599
uav1_in_uav0_yaw_deg = 30.0

uav0_in_uav1_x       = -3.098
uav0_in_uav1_y       = 0.634
uav0_in_uav1_z       = 0.0
uav0_in_uav1_yaw_deg = -30.0
```

错误的平移、角度单位或逆变换会同时造成远程PVBSM覆盖位置错误和EGO同伴轨迹“假碰撞”，这是双机联调中最需要优先排除的问题。

## 7. 首轮集中式启动步骤

### 7.1 UAV0启动ROS Master

UAV0容器：

```bash
source ~/daib_ws/devel/setup.bash
roscore
```

### 7.2 两机分别启动传感器驱动

在各自板卡上启动Livox、相机和IMU驱动。确保话题已命名为不同前缀，例如：

```text
/uav0/livox/lidar
/uav0/livox/imu
/uav0/camera/color/image_raw

/uav1/livox/lidar
/uav1/livox/imu
/uav1/camera/color/image_raw
```

检查：

```bash
rostopic hz /uav0/livox/lidar
rostopic hz /uav1/livox/lidar
rostopic hz /uav0/livox/imu
rostopic hz /uav1/livox/imu
```

### 7.3 两机分别启动FAST-LIVO2

UAV0容器：

```bash
roslaunch fast_livo mapping_mid70_d435i_daib_uav.launch \
  uav_name:=uav0 robot_id:=0 \
  lidar_topic:=/uav0/livox/lidar \
  imu_topic:=/uav0/livox/imu \
  image_topic:=/uav0/camera/color/image_raw
```

UAV1容器：

```bash
roslaunch fast_livo mapping_mid70_d435i_daib_uav.launch \
  uav_name:=uav1 robot_id:=1 \
  lidar_topic:=/uav1/livox/lidar \
  imu_topic:=/uav1/livox/imu \
  image_topic:=/uav1/camera/color/image_raw
```

如果D435i图像需经过已有预处理节点，应将`image_topic`替换为单机验证时FAST-LIVO2实际订阅的图像话题，不要因为双机部署绕过原有预处理。

确认两路LIVO输出：

```bash
rostopic hz /uav0/daib_slam/odom
rostopic hz /uav1/daib_slam/odom
rostopic hz /uav0/daib_slam/planning_cloud
rostopic hz /uav1/daib_slam/planning_cloud
```

### 7.4 仅在UAV0启动双机上层链路

零锚点示例：

```bash
roslaunch daib_explorer dual_uav_cooperation.launch \
  require_px4:=false \
  uav1_in_uav0_x:=0.0 \
  uav1_in_uav0_y:=0.0 \
  uav1_in_uav0_z:=0.0 \
  uav1_in_uav0_yaw_rad:=0.0 \
  uav1_in_uav0_yaw_deg:=0.0 \
  uav0_in_uav1_x:=0.0 \
  uav0_in_uav1_y:=0.0 \
  uav0_in_uav1_z:=0.0 \
  uav0_in_uav1_yaw_deg:=0.0
```

非零锚点必须同时填写第6节给出的正、逆变换。

该启动文件已经启动两套Explorer、两套Decision、两套Bridge、两套EGO-Planner和两个轨迹服务器。因此不要再并行启动单机的`explorer.launch`、`decision.launch`或`daib_single_uav.launch`。

## 8. 分阶段验收，不要直接起飞

### 阶段A：网络与命名空间

```bash
rosnode list | sort
rostopic list | grep -E 'uav0|uav1|daib_coexplore|broadcast_bspline'
```

必须看到两套不同命名空间，不应出现第二套无前缀的`/daib_slam/*`。

### 阶段B：坐标关系

```bash
rosrun tf tf_echo uav0/camera_init uav1/camera_init
```

输出应与第6节输入的锚点一致。把两架飞机静置在已知位置，检查两套里程计相对位置是否符合物理摆放。

### 阶段C：任务租约

```bash
rostopic echo /daib_coexplore/task
```

应交替看到`robot_id: 0`和`robot_id: 1`，每个来源约1 Hz更新。让两机面对相同Frontier时，只应有一机保留冲突目标；另一机重新选择目标。

若只出现一个`robot_id`，先检查另一机Explorer是否运行、时钟是否同步以及其TF能否转换到本机坐标。

### 阶段D：PVBSM多来源融合

```bash
rostopic hz /uav0/daib_slam/pvbsm_delta
rostopic hz /uav1/daib_slam/pvbsm_delta
rostopic hz /daib_coexplore/pvbsm_sync
rostopic hz /uav0/daib_coexplore/peer_pvbsm_delta
rostopic hz /uav1/daib_coexplore/peer_pvbsm_delta
rostopic echo /uav0/daib_explorer/pvbsm_memory_stats
rostopic echo /uav1/daib_explorer/pvbsm_memory_stats
```

`pvbsm_memory_stats`数组最后一个字段为`source_count`；两路增量都被接收后应为2。PVBSM只影响低频覆盖评分，不替代本机10 Hz占据图和碰撞判断。

两机对同一区域的覆盖结论不一致时，接收端采用保守值并增加重访倾向；该机制只处理
“覆盖事实冲突”，不等同于高精度点云级几何冲突融合。

### 阶段D2：20%应用层丢包恢复

Gazebo、SITL或双bag回放时，可用专用入口替代普通双机launch：

```bash
roslaunch daib_explorer dual_uav_cooperation_loss_sim.launch \
  loss_probability:=0.20
```

该参数只在PVBSM接收端注入丢包，不影响本机LIVO、占据地图和规划链。日志中应先出现`simulated drop`，随后出现缺口请求和缓存补发；最终两侧`source_count`均应恢复为2。也可先执行协议级自动测试：

```bash
rostest daib_explorer pvbsm_sync_relay.test
```

自动测试的通过判据不是“节点没有退出”，而是接收端最终按原顺序收到全部30个唯一序号。
任务租约本身以约1 Hz发布完整状态，因此单个租约包丢失可由下一次心跳覆盖；本入口当前
尚未对`/broadcast_bspline`单独注入应用层丢包。

### 阶段E：同伴轨迹

```bash
rostopic hz /broadcast_bspline
rostopic hz /uav0/daib_ego/position_cmd
rostopic hz /uav1/daib_ego/position_cmd
```

在SITL或卸桨条件下构造相交路径，确认EGO能够发现未来同一时刻的距离冲突并触发重规划。若两机相距很远仍持续报碰撞，优先检查静态锚点和角度单位。

### 阶段F：断链退化

停止UAV1的Explorer或断开其共享话题超过3 s：

1. UAV0 Decision应收到`PEER_LINK_LOST`；
2. UAV0本机LIVO、占据更新和局部规划应继续；
3. UAV1目标租约过期后，UAV0可以重新选择原冲突区域；
4. 恢复通信后，新的session/sequence应重新被接受，旧消息不得覆盖新状态。

## 9. 接入两套PX4

当前EGO输出分别为：

```text
/uav0/daib_ego/position_cmd
/uav1/daib_ego/position_cmd
```

它们不是PX4可直接执行的最终MAVROS命令。应继续使用单机已经验证的PX4适配节点，但必须启动两套实例，并分别绑定对应的MAVROS命名空间：

```text
UAV0 position_cmd → /uav0/mavros/*
UAV1 position_cmd → /uav1/mavros/*
```

严禁让UAV0适配器订阅UAV1指令或让两套适配器发布到同一个`/mavros/setpoint_*`。完成以下条件前保持`require_px4:=false`：

- 两机坐标变换已用实测位置验证；
- 两套position_cmd的frame、位置、速度和加速度方向正确；
- 单机失联、规划云陈旧和定位失效时能触发HOLD/降落安全动作；
- PX4 Offboard、解锁和failsafe已分别在两架飞机上验证；
- 卸桨测试中不存在跨机指令串线。

## 10. 常见问题定位

### 10.1 两套单机都正常，合起来没有对端话题

- 检查两机`ROS_MASTER_URI`是否完全相同；
- 检查`ROS_IP`是否分别为各自可达的物理网卡地址；
- Docker必须使用host网络；
- 用`rosnode info`检查节点公布的URI是否是对端可访问地址。

### 10.2 `/daib_coexplore/task`只有一架飞机

- 检查是否误启动了单机`explorer.launch`而非双机编排中的Explorer；
- 检查`cooperation/enabled`是否为true；
- 检查另一套Explorer是否收到自己的odom与planning cloud；
- 检查系统时间是否导致消息被判为未来或过期。

### 10.3 两机仍选择同一区域

- 检查任务消息中`robot_id`是否分别为0和1；
- 检查两机目标是否已转换到同一物理位置；
- 检查`exclusion_radius_m`是否满足场地尺度；默认2 m；
- 检查网络延迟是否超过3 s租约TTL。

### 10.4 PVBSM的`source_count`始终为1

- 检查两路`pvbsm_delta`是否都有数据；
- 检查FAST-LIVO2的`robot_id`是否重复；
- 检查远程消息frame是否存在对应TF；
- 检查是否错误地将两路PVBSM重映射到同一个本机来源话题。

### 10.5 EGO持续报告同伴碰撞

- 检查`yaw_rad`与`yaw_deg`是否混用；
- 检查逆变换是否按第6节计算，而不是简单对x、y取负；
- 检查两机时间同步；
- 检查旧的`/broadcast_bspline`发布节点是否仍在运行。

### 10.6 CPU占用突然翻倍

集中式入口会在UAV0启动两套Explorer、Decision和EGO，CPU升高是预期现象。先关闭RViz、高频日志和非必要点云显示；完成算法验证后应切换为双板分布式运行，而不是长期把两套规划器压在一块RK3588上。

## 11. 本轮联调记录建议

每次测试至少记录：

```text
测试日期与run_id
四个仓库Commit
两机IP和ROS_MASTER_URI
双机正、逆静态变换
两机系统时间偏差
任务消息robot_id/session/sequence
PVBSM source_count
目标冲突与重新选点日志
轨迹冲突与重规划日志
断链时间、PEER_LINK_LOST时间和恢复时间
两套position_cmd与PX4适配器日志
```

只有完成阶段A～F后，才进入低速、限高、带安全员的双机实飞测试。

## 12. 当前代码边界

本版已经实现目标租约、多来源PVBSM查询、对端健康状态、同伴轨迹检查、PVBSM序号缺口补发和失效目标区域接管，但以下能力尚未完成：

- 基于公共地图的在线跨机坐标自动对齐；
- 独立UDP分片、无线带宽整形和超出补发缓存后的全量快照回退；
- 完整Frontier簇拍卖和跨进程持久化区域任务账本；
- 可在两块板上分别一键启动的`daib_vehicle.launch`；
- DAIB输出到两套PX4的通用安全适配器与双机真机验收。

因此，当前最合理的部署路径是：**集中式无桨联调 → 双机SITL → 参数化分布式启动 → 两套PX4卸桨测试 → 低速实飞**。
