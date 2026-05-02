# 一、整体结构关系

```
config/
  └── requirement1_mission.yaml     # 任务一参数与航点配置

launch/
  └── requirement1_rflysim_fakeqr.launch
                                      # 任务一仿真启动入口

scripts/
  ├── requirement1_fsm_node.py       # 任务一核心状态机
  ├── mavros_sitl_bridge.py          # MAVROS 速度/模式桥
  ├── ground_udp_bridge_node.py      # 地面站 UDP 通信桥
  ├── sim_fake_qr_node.py            # 假二维码仿真节点
  ├── mavros_pose_to_odom.py         # pose 转 odom 备用节点
  └── save/新建 文本文档.txt         # 备份/临时文件，不参与运行
```

这三部分的关系可以概括为：

```
config 提供任务参数
launch 负责统一启动
scripts 执行具体逻辑
D_Ground 通过 UDP 与 scripts 中的通信桥联动
```

------

# 二、config 文件夹：任务参数与航点表

`config` 中只有：

```
requirement1_mission.yaml
```

它用于配置**要求一 / 任务一：遍历盘点**。

它不是控制程序，而是给 FSM 读取的**任务配置文件**，主要包括三类内容：

## 1. 飞行控制参数

包括：

```
起飞高度
到点容差
P 控制系数
速度限幅
偏航角速度限幅
```

这些参数决定无人机飞行过程中的运动表现。
 例如飞得快慢、是否容易超调、到什么误差范围内算到达航点等。

------

## 2. 视觉与对准参数

包括：

```
激光点图像像素坐标
二维码对准误差阈值
对准控制系数
对准速度限制
搜索超时
对准超时
激光点亮时间
地面站 ACK 超时
```

其中 `laser_u / laser_v` 表示的是**图像中的目标像素位置**，不是实际空间坐标。

这部分用于支持 FSM 在扫码点完成：

```
搜索二维码 → 调整姿态/高度对准 → 点亮激光 → 上报结果
```

------

## 3. 任务路径与航点

这是 YAML 里最关键的部分。

它定义了：

```
降落点
safe 安全过渡点
scan 盘点点
A/B/C/D 四个货架面
每个面 1~6 号槽位
每个点的 x / y / z / yaw
```

其中：

```
safe：用于绕行、转场，降低碰撞风险
scan：真正执行二维码盘点的位置
face：货架面，如 A/B/C/D
slot：该面上的固定位置，如 1~6
```

例如某个 `scan` 点写着 `face: A, slot: 1`，那么 FSM 飞到这个点并识别到二维码编号 17 后，就会形成：

```
A1 对应 17
```

然后再通过通信桥发给地面站。

------

# 三、launch 文件夹：仿真启动入口

`launch` 中只有：

```
requirement1_rflysim_fakeqr.launch
```

它的定位是：

```
要求一 / 任务一的 RflySim 仿真联调启动文件
```

注意，它更偏向**仿真测试**，不是最终真机完整启动文件。
 从名字里的 `rflysim_fakeqr` 就能看出来，它包含 RflySim 和假二维码节点。

它主要做五件事：

## 1. 加载 YAML 参数

```
加载 config/requirement1_mission.yaml
```

把任务参数加载进 ROS 参数服务器，供 FSM 使用。

需要注意：

```
$(find uav_inventory)
```

说明它默认 ROS 包名是：

```
uav_inventory
```

如果后续建包时包名不同，launch 文件需要同步修改。

------

## 2. 启动 MAVROS 速度桥

启动：

```
mavros_sitl_bridge.py
```

作用是把 FSM 发出的：

```
/uav/cmd_vel
```

转发给 MAVROS：

```
/mavros/setpoint_velocity/cmd_vel_unstamped
```

也就是：

```
FSM → /uav/cmd_vel → MAVROS 速度桥 → MAVROS → 飞控/仿真机
```

------

## 3. 启动假二维码节点

启动：

```
sim_fake_qr_node.py
```

用于在没有真实相机和二维码识别程序时，模拟发布：

```
/qr/result
```

方便测试 FSM 的完整流程。

真机阶段，这个节点应被真实视觉节点替代。

------

## 4. 启动任务一 FSM

启动：

```
requirement1_fsm_node.py
```

这是任务一主状态机，负责起飞、巡航、扫码、点激光、上报结果和降落。

当前 launch 中，FSM 的里程计话题被配置为：

```
/mavros/local_position/odom
```

因此它不依赖 `mavros_pose_to_odom.py`。

------

## 5. 启动地面站 UDP 通信桥

启动：

```
ground_udp_bridge_node.py
```

这是和 `D_Ground` 联动的关键节点。

它监听机载端 UDP 端口：

```
8889
```

地面站 `D_Ground` 监听：

```
8888
```

基本通信方向是：

```
D_Ground → CMD 指令 → ground_udp_bridge_node.py → ROS 话题 → FSM
FSM → 状态/盘点结果 → ground_udp_bridge_node.py → UDP 消息 → D_Ground
```

------

# 四、scripts 文件夹：核心运行逻辑

`scripts` 是最重要的目录。
 它包含真正运行的 ROS 节点。

------

## 1. `requirement1_fsm_node.py`：任务一核心状态机

这是整个机载端任务一的核心。

它读取：

```
/mission 参数
```

也就是 `requirement1_mission.yaml` 中的航点和控制参数。

它订阅：

```
odom_topic 指定的里程计话题
/qr/result
/uav/start
/uav/stop
/ground/ack
```

其中里程计话题默认可能是 `/Odometry`，但当前 launch 中配置为：

```
/mavros/local_position/odom
```

它发布：

```
/uav/cmd_vel
/laser/cmd
/inventory/result
/fsm/state
```

核心状态流程是：

```
IDLE
→ TAKEOFF
→ GOTO_MISSION_POINT
→ SEARCH_QR
→ ALIGN_QR
→ LASER_FLASH
→ SEND_RESULT
→ NEXT_POINT
→ RETURN_LAND
→ LAND
→ FINISH
```

同时包含：

```
EMERGENCY_STOP
```

用于急停处理。

它的核心逻辑是：

```
起飞后记录起飞点
按 YAML 中的航点顺序飞行
到 safe 点只做过渡
到 scan 点执行二维码搜索和对准
对准后点亮激光
将 face / slot / 编号组成盘点结果
发布 /inventory/result
继续下一个点
最后返航降落
```

其中 ACK 不会长期阻塞流程。
 如果扫码、对准或 ACK 超时，FSM 会记录失败或继续后续任务，避免整套流程卡死。

------

## 2. `mavros_sitl_bridge.py`：速度与模式桥

它负责连接 FSM 和 MAVROS。

FSM 不直接控制飞控，而是发布：

```
/uav/cmd_vel
```

`mavros_sitl_bridge.py` 再把它转发给：

```
/mavros/setpoint_velocity/cmd_vel_unstamped
```

它还处理：

```
/uav/start
/uav/stop
```

收到 start 后：

```
尝试切 OFFBOARD
尝试解锁
开始持续发送速度 setpoint
```

收到 stop 后：

```
发布零速度
尝试调用 MAVROS 降落服务
```

同时它带有速度指令超时保护。
 如果一段时间内没有收到新的 `/uav/cmd_vel`，它会输出零速度，防止无人机沿用旧指令继续运动。

------

## 3. `ground_udp_bridge_node.py`：D_Ground 通信桥

这是 `scripts` 中和地面站直接相关的文件。

它一边收发 UDP，一边发布/订阅 ROS 话题。

地面站发来的命令例如：

```
CMD:START_TASK1
CMD:START_TASK2
CMD:TASK2_SCAN_TARGET
CMD:EMERGENCY_STOP
```

会被它转换成 ROS 话题：

```
CMD:START_TASK1        → /uav/start
CMD:EMERGENCY_STOP     → /uav/stop
CMD:START_TASK2        → /uav/start_task2
CMD:TASK2_SCAN_TARGET  → /uav/scan_target
```

它也会把 ROS 端的信息转成地面站协议：

```
/fsm/state        → STATUS:xxx
/inventory/result → SCAN:A1:17
/target/id        → TARGET_ID:17
/target/result    → TARGET_RESULT:C4:17:OK
```

所以它的本质是：

```
UDP 协议 ↔ ROS 话题
```

它是 `D_Ground` 和机载 ROS 系统之间的翻译层。

------

## 4. `sim_fake_qr_node.py`：假二维码仿真节点

这个节点用于仿真阶段。

它监听：

```
/fsm/state
```

当 FSM 进入搜索、对准、点激光、发送结果等阶段时，它模拟发布：

```
/qr/result
```

这样即使没有真实相机，FSM 也能继续执行完整的盘点流程。

它适合：

```
无相机测试
无二维码识别算法测试
FSM 流程验证
地面站联调
```

但真机阶段应替换为真实视觉节点。

真实视觉节点最好保持同样的输出话题：

```
/qr/result
```

这样 FSM 不需要大改。

------

## 5. `mavros_pose_to_odom.py`：备用位姿转换节点

它的作用是把：

```
/mavros/local_position/pose
```

转换成：

```
/Odometry
```

供 FSM 使用。

但是当前 launch 文件没有启动它。
 因为当前 FSM 直接订阅的是：

```
/mavros/local_position/odom
```

所以它目前属于备用适配脚本。

只有在实际环境中没有 `/mavros/local_position/odom`，但有 `/mavros/local_position/pose` 时，才需要启动它。

------

## 6. `scripts/save/新建 文本文档.txt`

这个文件没有被 launch 调用，不参与运行。
 它更像是备份文件或误放的临时文件，不应作为正式节点使用。

------

# 五、三部分如何和 D_Ground 联动

整体联动链路如下：

```
D_Ground 用户点击“任务1”
        ↓
发送 UDP：CMD:START_TASK1
        ↓
ground_udp_bridge_node.py 接收
        ↓
发布 ROS 话题：/uav/start
        ↓
requirement1_fsm_node.py 开始任务
        ↓
读取 requirement1_mission.yaml 中的航点
        ↓
发布 /uav/cmd_vel
        ↓
mavros_sitl_bridge.py 转发给 MAVROS
        ↓
无人机 / 仿真机运动
```

扫码结果回传链路是：

```
二维码识别节点 / sim_fake_qr_node.py
        ↓
发布 /qr/result
        ↓
requirement1_fsm_node.py 生成盘点结果
        ↓
发布 /inventory/result
        ↓
ground_udp_bridge_node.py 转成 UDP
        ↓
发送 SCAN:A1:17
        ↓
D_Ground 更新表格并保存结果
```

状态回传链路是：

```
requirement1_fsm_node.py
        ↓
发布 /fsm/state
        ↓
ground_udp_bridge_node.py
        ↓
发送 STATUS:TAKEOFF / STATUS:SCANNING / STATUS:LANDING 等
        ↓
D_Ground 更新状态灯和日志
```

------

# 六、核心理解

这套机载端代码可以分成三层：

```
第一层：任务配置层
config/requirement1_mission.yaml
决定飞哪里、扫哪里、每个位置对应 A1~D6 哪个坐标。

第二层：任务执行层
requirement1_fsm_node.py
负责起飞、巡航、扫码、点激光、上报、降落。

第三层：接口桥接层
ground_udp_bridge_node.py 负责和 D_Ground 通信；
mavros_sitl_bridge.py 负责和 MAVROS/飞控通信；
sim_fake_qr_node.py 负责仿真二维码输入。
```

最终概括：

**`config` 定义任务，`launch` 负责启动，`scripts` 执行控制与通信；其中 `requirement1_fsm_node.py` 是任务核心，`ground_udp_bridge_node.py` 是和 D_Ground 联动的关键，`mavros_sitl_bridge.py` 是和飞控/MAVROS 联动的关键。**