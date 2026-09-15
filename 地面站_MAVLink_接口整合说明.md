# DYT / Cooperative Rendezvous 地面站 MAVLink 接口整合说明

本文档按当前飞控源码整理，供地面站开发、联调和协议生成使用。内容覆盖：

- `cooperative_rendezvous` 中制导/协同会合；
- DYT 导引头遥测、状态回复和内部控制能力；
- `dyt_guidance` 中末制导切换、制导状态和控制量；
- 前视激光测距自动网捕、手动网捕及网捕后制动/悬停；
- 下视激光测距降落与快速触地；
- 通信中断和 GNSS 异常应急策略；
- 当前地面站能够直接使用的 MAVLink 消息，以及尚未暴露到 MAVLink 的内部状态。

> 重要：本文档以当前 `src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml` 为准。旧文档中的 `DYT_TARGET(12932)`、`DYT_COMMAND(12933)`、`DYT_GUIDANCE_STATUS(12934)` 已与当前固件不一致，不能继续用于生成地面站 MAVLink 库。

## 1. 协议版本和接入约定

### 1.1 MAVLink 版本

- 自定义消息 ID 大于 255，必须使用 **MAVLink 2**。
- 地面站和飞控必须使用同一份修改后的 `common.xml` 生成代码，否则消息 ID、字段布局和 CRC 不匹配。
- 当前主 dialect 为 `common`。
- 多机情况下以 MAVLink 包头的 `sysid` 区分飞机；不要只依赖消息体内的 `mavid`。
- `DYT_SYSTEM_STATUS` 的原始 Payload 为 91 B；新模式反馈位于 MAVLink 2 扩展区，最大 Payload 为 93 B，CRC Extra 仍为 `0`。
- 不建议地面站手工按 XML 顺序拼 payload；MAVLink 生成器会按字段类型重排线上的字段布局。

生成 C 头文件示例：

```sh
python3 pymavlink/tools/mavgen.py \
  --lang=C \
  --wire-protocol=2.0 \
  --output=generated \
  message_definitions/v1.0/common.xml
```

### 1.2 当前飞机角色约定

| MAV_SYS_ID | 默认角色 | cooperative_rendezvous 启动角色 | `vehicle_type` 默认值 |
| ---: | --- | --- | ---: |
| `1` | 格斗机/目标机 | `broadcast` | `1` Fighter |
| `2` | 网捕机/会合机 | `rendezvous`，目标 ID 为 1 | `2` Net Capture |
| 其他 | 按参数配置 | 未自动启动固定角色 | 默认 `1` Fighter |

`DYT_VEH_TYPE` 可覆盖上述自动映射：`0=按 MAV_SYS_ID 自动判断`、`1=格斗机`、`2=网捕机`。

### 1.3 消息方向标记

| 标记 | 含义 |
| --- | --- |
| `GCS -> FC` | 地面站发送给飞控 |
| `FC -> GCS` | 飞控发送给地面站 |
| `FC <-> FC` | 多机之间广播/转发，地面站也可以监听 |
| `Sensor -> FC` | 外部传感器通过 MAVLink 输入飞控 |

## 2. 消息总览

### 2.1 自定义 MAVLink 消息

| ID | Name | Direction | Payload | CRC Extra | 默认频率/触发方式 | 用途 |
| ---: | --- | --- | ---: | ---: | --- | --- |
| `12921` | `UAV_INFO` | `FC <-> FC`，GCS 可监听 | 45 B | 100 | 10 Hz；主机可能每周期发送两包 | 协同位置、速度、角色、降落/到位状态 |
| `12922` | `LEADER_GROUP_ID` | `FC -> GCS/FC` | 5 B | 153 | 1 Hz，只有 leader=true 时发送 | 组号和主机标志 |
| `12923` | `SWARM_START_FLAG` | `GCS -> FC` | 5 B | 125 | 命令触发 | 集群起飞/降落/暂停/继续 |
| `12924` | `SWARM_OPERATION_ACK` | `FC -> GCS` | 19 B | 71 | 状态更新触发，流上限 10 Hz | 集群操作结果 |
| `12925` | `DYT_GUIDANCE_COMMAND` | `GCS -> FC` | 7 B | 169 | 命令触发 | 请求进入中制导或末制导 |
| `12926` | `DYT_SYSTEM_STATUS` | `FC -> GCS` | 91–93 B | 0 | DYT telemetry 流 10 Hz | 飞机类型、控制模式、制导阶段、命令结果、网捕状态、末制导控制量 |
| `12927` | `DYT_TARGET_STATUS` | `FC -> GCS` | 83 B | 157 | DYT telemetry 流 10 Hz，上游更新触发 | 导引头目标、云台和链路遥测 |
| `12928` | `DYT_STATUS_REPLY` | `FC -> GCS` | 29 B | 227 | 导引头回复更新触发 | 导引头原始命令回复 |
| `12931` | `SWARM_MISSION_ITEM` | `FC <-> FC`，GCS 可收发 | 60 B | 39 | 5 Hz，上游更新触发 | 集群任务航点同步 |

`12929` 当前未定义；`12930 TEST_MAVLINK` 仅用于测试，不属于业务接口。

### 2.2 本项目依赖的标准 MAVLink 消息

| ID | Name | Direction | 主要用途 |
| ---: | --- | --- | --- |
| `0` | `HEARTBEAT` | `FC <-> GCS` | 在线状态、解锁位和 PX4 飞行模式；应急模式结果也从这里观察 |
| `76` | `COMMAND_LONG` | `GCS -> FC` | 标准解锁、模式、RTL、LAND、网捕器等命令微服务 |
| `77` | `COMMAND_ACK` | `FC -> GCS` | 标准命令执行结果 |
| `132` | `DISTANCE_SENSOR` | `Sensor -> FC`、`FC -> GCS` | 下视激光降落和前视激光网捕距离 |
| `147` | `BATTERY_STATUS` | `FC -> GCS` | 通信中断应急中的电量和剩余飞行时间依据 |
| `245` | `EXTENDED_SYS_STATE` | `FC -> GCS` | 空中、正在降落、已落地状态 |
| `253` | `STATUSTEXT` | `FC -> GCS` | 模式拒绝、应急动作和模块告警文本 |

## 3. Cooperative Rendezvous / 集群接口

## UAV_INFO (12921)

协同会合和多机避撞位置消息。`cooperative_rendezvous` 使用接收到的真实目标位置产生中制导轨迹；原集群模块还复用该消息传输主机轨迹设定点。

### 字段表

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `mavid` | `uint32_t` |  | 发送飞机 ID。`>=100` 是本项目的特殊编码，见下文。 |
| `group_id` | `uint32_t` |  | 集群组号。 |
| `is_leader` | `uint8_t` | `0/1` | 是否为主机。 |
| `lat` | `float` | `deg` | 纬度。注意线上的类型是 float，不是 double。 |
| `lon` | `float` | `deg` | 经度。注意线上的类型是 float，不是 double。 |
| `yaw` | `float` | `rad` 或特殊编码 | 正常为航向；主机目标包可能使用 `home.z + 1000` 特殊编码。 |
| `yaw_speed` | `float` | `rad/s` | yaw 角速度。 |
| `rel_alt` | `float` | `m AMSL` | 字段名保留为 rel_alt，但当前代码实际统一按 AMSL 高度使用。 |
| `vx` | `float` | `m/s` | NED 北向速度或目标速度。 |
| `vy` | `float` | `m/s` | NED 东向速度或目标速度。 |
| `vz` | `float` | `m/s` | NED 下向速度，向下为正。 |
| `land` | `uint32_t` | bitmask | Bit 0 正在降落；Bit 1 已到达目标位置。 |

### mavid 特殊编码

| 条件 | 含义 | 飞控接收后的 uORB |
| --- | --- | --- |
| `is_leader=1` 且 `mavid<100` | 主机目标轨迹/设定点 | `uav_info`，供传统从机跟随 |
| `is_leader=1` 且 `mavid>=100` | 主机真实位置，真实 ID 为 `mavid-100` | `follower_info`，用于避撞和 cooperative target |
| `is_leader=0` | 从机真实位置 | `follower_info` |

主机通常每个发送周期发两包：一包 `mavid=真实 ID` 的目标轨迹，一包 `mavid=真实 ID+100` 的真实位置。地面站显示飞机时应把 `+100` 包归并到原飞机，不能创建一架新的飞机。

### land bitmask

| Bit | Mask | Name | Description |
| ---: | ---: | --- | --- |
| 0 | `0x01` | `LANDING` | 飞机 `nav_state` 为 AUTO_LAND。 |
| 1 | `0x02` | `AT_TARGET` | 从机水平速度小于 0.5 m/s，代码将其视作已到位。 |

### cooperative_rendezvous 处理逻辑

1. 接收真实位置类型的 `UAV_INFO`，转换成 `follower_info`。
2. 目标位置必须新鲜，且纬度、经度和 AMSL 高度有效。
3. 将全局位置投影到本机 NED；`CRDZ_XY_OFF_EN=1` 时，按目标运动方向旋转并叠加 `CRDZ_FB_OFF/CRDZ_LR_OFF`；否则使用 `CRDZ_DIST` 在目标后方保持距离；最后应用 `CRDZ_ALT_DIFF`。
4. 产生位置和速度 setpoint，并请求 OFFBOARD。
5. DYT 末制导真正接管飞机后，cooperative 模块停止发布轨迹 setpoint；丢锁搜索期间仍由 cooperative 控制飞机。

### 关键参数

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `CRDZ_ACT_AUX` | 3 |  | cooperative 一次性激活 AUX；从低到高触发，-1/0 禁用。 |
| `CRDZ_ACT_BTN` | -1 |  | QGC 一次性触发按钮；退出后必须松开再按，-1 禁用。 |
| `CRDZ_DIST` | 0 | m | 相对目标水平距离。 |
| `CRDZ_XY_OFF_EN` | 0 | bool | 使用目标随动的前后/左右偏移。 |
| `CRDZ_FB_OFF` | -5 | m | 目标前后偏移：负数在后方，正数在前方。 |
| `CRDZ_LR_OFF` | 0 | m | 目标左右偏移：负数在左方，正数在右方。 |
| `CRDZ_ALT_DIFF` | 0 | m | 正值表示会合机高于目标机。 |
| `CRDZ_APP_SPD` | 4 | m/s | 接近目标的附加闭合速度。 |
| `CRDZ_SLOW_RAD` | 5 | m | 接近目标时开始线性减速的半径。 |
| `CRDZ_ALT_ERR` | 100 | m | 允许的最大垂向 setpoint 误差。 |
| `CRDZ_HOLD_EN` | 0 | bool | 到位后保持/伴飞逻辑。 |
| `CRDZ_VSLEW` | 3 | m/s² | 水平速度变化率限制。 |
| `CRDZ_TPOS_TC` | 1 | s | 目标位置低通时间常数。 |
| `CRDZ_TVEL_TC` | 0.5 | s | 目标速度低通时间常数。 |
| `CRDZ_TPOS_JMP` | 7 | m | 单次目标位置跳变限制。 |

## LEADER_GROUP_ID (12922)

| Field Name | Type | Description |
| --- | --- | --- |
| `group_id` | `uint32_t` | 主机所在组号。 |
| `leader` | `uint8_t` | `1` 表示本机为 leader。当前发送端只有 leader=true 时才发送。 |

## SWARM_START_FLAG (12923)

地面站向一组或全部飞机发送集群操作。当前代码复用了 `start_swarm` 字段作为目标组号。

| Field Name | Type | Current Meaning |
| --- | --- | --- |
| `start_swarm_auto` | `uint8_t` | 起飞/进入自动集群流程标志。 |
| `start_swarm` | `uint8_t` | **目标组号**：0=全部组，1–4=指定组；不再是简单 bool。 |
| `stop_swarm` | `uint8_t` | 降落/停止集群。 |
| `Pause_swarm` | `uint8_t` | 暂停。字段名首字母大写，生成代码时必须保持。 |
| `continue_swarm` | `uint8_t` | 继续。 |

## SWARM_OPERATION_ACK (12924)

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `timestamp` | `uint64_t` | us | PX4 boot time。 |
| `target_system` | `uint8_t` |  | 执行操作的飞机 sysid。 |
| `operation_type` | `uint8_t` |  | 操作类型。 |
| `result` | `uint8_t` |  | `0=SUCCESS`，`1=FAILED`。 |
| `old_value` | `uint32_t` |  | 操作前的值。 |
| `new_value` | `uint32_t` |  | 操作后的值。 |

| operation_type | Name |
| ---: | --- |
| 1 | GROUP_CHANGE |
| 2 | LEADER_CHANGE |
| 3 | TAKEOFF |
| 4 | LAND |
| 5 | PAUSE |
| 6 | CONTINUE |

## SWARM_MISSION_ITEM (12931)

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `timestamp` | `uint64_t` | us | PX4 boot time。 |
| `group_id` | `uint8_t` |  | 目标集群组号。 |
| `leader_id` | `uint8_t` |  | 主机 ID。 |
| `mission_id` | `uint32_t` |  | 任务版本/更新 ID。 |
| `total_count` | `uint16_t` |  | 总航点数。 |
| `current_seq` | `uint16_t` |  | 当前执行航点。 |
| `seq` | `uint16_t` |  | 本包航点序号。 |
| `nav_cmd` | `uint16_t` |  | MAVLink/PX4 导航命令 ID。 |
| `lat` | `double` | deg | 纬度。 |
| `lon` | `double` | deg | 经度。 |
| `alt` | `float` | m AMSL | 海拔高度。 |
| `yaw` | `float` | rad | NED yaw。 |
| `acceptance_radius` | `float` | m | 航点接受半径。 |
| `loiter_radius` | `float` | m | 盘旋半径。 |
| `time_inside` | `float` | s | 航点停留时间。 |
| `autocontinue` | `uint8_t` | 0/1 | 自动继续。 |
| `sync_type` | `uint8_t` |  | `0=SINGLE, 1=START, 2=END, 3=CLEAR`。 |

## 4. DYT 制导控制与状态

## DYT_GUIDANCE_COMMAND (12925)

地面站请求中制导或末制导。该消息只控制制导阶段，不直接控制导引头的可见光、红外、激光或跟踪器命令。

### 字段表

| Field Name | Type | Values | Description |
| --- | --- | --- | --- |
| `request_id` | `uint32_t` | 单调递增 | 地面站生成的请求序号；用于和状态回复关联。 |
| `target_system` | `uint8_t` | 0 或 sysid | 目标飞机；0 为广播。多机控制建议禁止使用 0。 |
| `target_component` | `uint8_t` | 0 或 compid | 目标组件；0 表示 autopilot/all component。 |
| `phase` | `uint8_t` | 2/3 | 请求阶段：2 中制导，3 末制导。 |

### phase 枚举

| Value | Name | Current Action |
| ---: | --- | --- |
| `2` | `PHASE_MIDCOURSE` | cooperative_rendezvous 控制飞机，DYT 根据共享目标做地理/角度指向。 |
| `3` | `PHASE_TERMINAL` | 授权 DYT 末制导，等待目标锁定后接管飞机。 |

半自动模式 `DYTG_MODE=1` 下，点选锁定不会清除中制导，也不会自动进入末制导。地面站必须等待 `DYT_SYSTEM_STATUS.semi_auto_state=3`，再发送一条新的 `DYT_GUIDANCE_COMMAND(12925)`，其中 `phase=3`；只有该确认命令被接受后，才允许从中制导切换到末制导。

### 地面站发送规则

1. `request_id` 每次新请求加 1；不要反复用同一个值发不同命令。
2. 最少重发 3 次或重发至看到同一 `command_sequence`，建议间隔 200–500 ms。
3. 收到 `DYT_SYSTEM_STATUS.command_sequence == request_id` 后读取 `command_result`。
4. `PENDING` 表示飞控已接收并等待模块状态变化；`ACCEPTED` 是当前代码认定的切换完成。
5. 当前协议没有 phase=0/取消命令；退出依赖新阶段请求、人工接管、failsafe 或解除锁定。

## DYT_SYSTEM_STATUS (12926)

统一返回飞机类型、全局制导阶段、最新 GCS 指令结果、网捕触发状态以及末制导详细控制量。

### 字段表

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `time_boot_ms` | `uint32_t` | ms | PX4 boot time。 |
| `command_sequence` | `uint32_t` |  | 最近处理的 `DYT_GUIDANCE_COMMAND.request_id`。 |
| `net_trigger_count` | `uint32_t` |  | 本次开机以来飞控发布网捕释放命令的累计次数。 |
| `los_age_s` | `float` | s | 最新 LOS 样本年龄；无样本时可为 NaN。 |
| `frame_dt_s` | `float` | s | DYT 目标帧间隔。 |
| `delay_s` | `float` | s | 末制导固定管线延迟参数。 |
| `los_ned[3]` | `float[3]` | unit vector | NED LOS 向量 `[North, East, Down]`。 |
| `omega_los_ned[3]` | `float[3]` | rad/s | NED LOS 角速度。 |
| `velocity_sp[3]` | `float[3]` | m/s | NED 速度 setpoint，Z 向下为正。 |
| `acceleration_sp[3]` | `float[3]` | m/s² | NED 加速度前馈。 |
| `yaw_sp` | `float` | rad | yaw setpoint。 |
| `yaw_rate_sp` | `float` | rad/s | yaw-rate setpoint。 |
| `status_flags` | `uint16_t` | bitmask | 综合状态位，见下表。 |
| `vehicle_type` | `uint8_t` | enum | 飞机类型。 |
| `control_mode` | `uint8_t` | enum | `0=手动`、`1=半自动`、`2=全自动`。 |
| `semi_auto_state` | `uint8_t` | enum | `0=禁用`、`1=空闲`、`2=已点选/请求锁定`、`3=已锁定等待确认`、`4=制导已激活`。 |
| `guidance_phase` | `uint8_t` | enum | 飞控实际判断的全局导引阶段。 |
| `gcs_phase_request` | `uint8_t` | enum | 当前保留的 GCS 阶段请求，0 表示没有。 |
| `command_phase` | `uint8_t` | enum | 最近一条 GCS 命令携带的阶段。 |
| `command_result` | `uint8_t` | enum | 最近命令结果。 |
| `guidance_state` | `uint8_t` | enum | DYT 末制导内部状态机。 |
| `requested_submode` | `uint8_t` | enum | 请求的末制导子模式。 |
| `active_submode` | `uint8_t` | enum | 实际生效的末制导子模式。 |
| `lost_reason` | `uint8_t` | enum | 最近丢锁/退出原因。 |

### vehicle_type

| Value | Name | Description |
| ---: | --- | --- |
| 0 | UNKNOWN | XML 保留值；当前自动映射实际不会输出该值。 |
| 1 | FIGHTER | 格斗机/目标机。 |
| 2 | NET_CAPTURE | 网捕机。 |

### guidance_phase

| Value | Name | Current Code Meaning |
| ---: | --- | --- |
| 0 | DISARMED | 未解锁。 |
| 1 | INITIAL | 已解锁，且当前既未进入 cooperative active，也未进入 DYT 末制导状态机。当前代码不严格限定为 Mission Takeoff。 |
| 2 | MIDCOURSE | cooperative status 新鲜且 active。 |
| 3 | TERMINAL | DYT 状态机不处于 IDLE/ABORT，包括等待锁定和丢锁保持。 |

### command_result

| Value | Name | Description |
| ---: | --- | --- |
| 0 | NONE | 尚未处理 GCS 制导命令。 |
| 1 | PENDING | 已接收，请求正在生效。 |
| 2 | ACCEPTED | 当前代码认为阶段切换成功。 |
| 3 | DENIED | 阶段值非法或解锁、位置、failsafe 等前置条件不满足。 |
| 4 | FAILED | 执行中失去解锁/failsafe，或 3 秒未完成。 |

### guidance_state

| Value | Name | Description |
| ---: | --- | --- |
| 0 | IDLE | DYT 制导空闲。 |
| 1 | SEARCH_WAIT_LOCK | 末制导已授权，等待导引头锁定。 |
| 2 | TRACK_FOLLOW | 视觉 Follow。 |
| 3 | TRACK_INTERCEPT | 视觉 Intercept。 |
| 4 | LOST_HOLD | 丢锁保持/重捕。 |
| 5 | ABORT | 中止，下一周期返回 IDLE。 |

### submode

| Value | Name | Description |
| ---: | --- | --- |
| 0 | FOLLOW | 稳定逼近/伴飞。 |
| 1 | INTERCEPT | 快速闭合/末端拦截。 |

### lost_reason

| Value | Name | Description |
| ---: | --- | --- |
| 0 | NONE | 无。 |
| 1 | TRACKING | 跟踪状态丢失。 |
| 2 | TIMEOUT | 等待锁定或丢锁等待超时。 |
| 3 | STALE | 目标数据陈旧。 |
| 4 | MANUAL | 人工摇杆接管。 |
| 5 | PRECONDITION | 解锁、位置或 failsafe 前置条件失败。 |
| 6 | JITTER | 帧间隔/链路抖动异常。 |

### status_flags

| Bit | Mask | Name | Description |
| ---: | ---: | --- | --- |
| 0 | `0x0001` | ARMED | `guidance_phase != 0`；用于显示时仍建议以 HEARTBEAT armed bit 为准。 |
| 1 | `0x0002` | GUIDANCE_ACTIVE | DYT 末制导状态机 active。 |
| 2 | `0x0004` | TARGET_LOCKED | 导引头报告锁定。 |
| 3 | `0x0008` | DYT_CONTROLS_VEHICLE | DYT 正在发布飞机轨迹 setpoint。 |
| 4 | `0x0010` | TARGET_FRESH | 目标数据满足新鲜度要求。 |
| 5 | `0x0020` | INTERCEPT_ALLOWED | 当前允许进入 Intercept。 |
| 6 | `0x0040` | MIDCOURSE_ACTIVE | cooperative 中制导 active。 |
| 7 | `0x0080` | MIDCOURSE_TARGET_VALID | 中制导目标位置新鲜有效。 |
| 8 | `0x0100` | NET_TRIGGER_SENT | 当前末制导会话已经发送过网捕释放命令。 |

## 5. DYT 导引头遥测和回复

## DYT_TARGET_STATUS (12927)

### 枚举

| Field | Value | Name | Description |
| --- | ---: | --- | --- |
| tracking_state | 0 | SEARCH | 搜索/未锁定。 |
| tracking_state | 1 | LOCKED | 已锁定。 |
| tracking_state | 2 | TIMEOUT | 吊舱遥测超时。 |
| tracking_state | 3 | ERROR | 协议或解析错误。 |
| video_source | 0 | VIS_1 | 可见光 1。 |
| video_source | 1 | VIS_2 | 可见光 2。 |
| video_source | 2 | IR_1 | 红外 1。 |
| video_source | 3 | IR_2 | 红外 2。 |
| tracking_algorithm | 0 | ADAPTIVE | 自适应。 |
| tracking_algorithm | 1 | PERSON | 人员。 |
| tracking_algorithm | 2 | VEHICLE | 车辆。 |
| tracking_algorithm | 3 | BUILDING | 建筑。 |

### 字段表

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `time_boot_ms` | `uint32_t` | ms | 飞控发布时刻。 |
| `time_sample_ms` | `uint32_t` | ms | 导引头遥测采样/接收时刻。 |
| `frame_counter` | `uint32_t` |  | DYT 帧计数。 |
| `los_x_rad` | `float` | rad | 图像横向 LOS 误差。 |
| `los_y_rad` | `float` | rad | 图像纵向 LOS 误差。 |
| `gimbal_roll_rad` | `float` | rad | 云台 roll。 |
| `gimbal_pitch_frame_rad` | `float` | rad | 框架 pitch，协议字节 12–13。 |
| `gimbal_pitch_rad` | `float` | rad | 姿态 pitch，协议字节 24–25。 |
| `gimbal_yaw_rad` | `float` | rad | 云台 yaw。 |
| `gimbal_roll_rate_rad_s` | `float` | rad/s | roll 角速度。 |
| `gimbal_pitch_rate_rad_s` | `float` | rad/s | pitch 角速度。 |
| `gimbal_yaw_rate_rad_s` | `float` | rad/s | yaw 角速度。 |
| `bbox_width_px` | `float` | pixel | 目标框宽度。 |
| `bbox_height_px` | `float` | pixel | 目标框高度。 |
| `range_m` | `float` | m | 导引头自身测距；不可用时可能是 NaN。不要和前视 SDM50 混淆。 |
| `zoom_ratio` | `float` |  | 变焦倍率。 |
| `frame_dt_s` | `float` | s | 相邻目标帧间隔。 |
| `last_rx_age_s` | `float` | s | 最近一次导引头串口数据年龄。 |
| `parse_error_count` | `uint16_t` |  | 累计解析错误数。 |
| `target_flags` | `uint16_t` | bitmask | 目标/吊舱状态位。 |
| `tracking_state` | `uint8_t` | enum | 跟踪状态。 |
| `video_source` | `uint8_t` | enum | 当前视频源。 |
| `tracking_algorithm` | `uint8_t` | enum | 跟踪算法。 |
| `status1` | `uint8_t` | raw | 原始状态字 1。 |
| `status2` | `uint8_t` | raw | 原始状态字 2。 |
| `status3` | `uint8_t` | raw | 原始状态字 3。 |
| `self_test_raw` | `uint8_t` | raw | 原始自检字。 |

### target_flags

| Bit | Mask | Name |
| ---: | ---: | --- |
| 0 | `0x0001` | TARGET_VALID |
| 1 | `0x0002` | AUTO_HINT |
| 2 | `0x0004` | IMAGE_ENHANCE |
| 3 | `0x0008` | RECORDING |
| 4 | `0x0010` | MOTOR_ON |
| 5 | `0x0020` | FOLLOW_MODE |
| 6 | `0x0040` | LASER_ON |
| 7 | `0x0080` | SELFTEST_DONE |
| 8 | `0x0100` | GYRO_CALIB_FAILED |
| 9 | `0x0200` | SERVO_FAULT |
| 10 | `0x0400` | IMAGE_BOARD_FAULT |

## DYT_STATUS_REPLY (12928)

导引头返回的通用原始回复。它不是 `DYT_GUIDANCE_COMMAND` 的切换应答；制导切换应答在 `DYT_SYSTEM_STATUS` 中。

| Field Name | Type | Units | Description |
| --- | --- | --- | --- |
| `time_boot_ms` | `uint32_t` | ms | 飞控发布时刻。 |
| `time_sample_ms` | `uint32_t` | ms | 回复帧接收时刻。 |
| `parse_error_count` | `uint16_t` |  | 累计解析错误。 |
| `control_code` | `uint8_t` | raw | DYT 回复控制码。 |
| `param_length` | `uint8_t` | byte | `params` 中有效字节数。 |
| `truncated` | `uint8_t` | 0/1 | 原回复是否超过 16 字节而被截断。 |
| `params[16]` | `uint8_t[16]` | raw | 原始回复参数。 |

地面站解析时只能读取 `params[0..min(param_length,16)-1]`。

## 6. DYT 导引头全部内部命令能力

当前 `DytCommand.msg` 定义了以下命令，导引头驱动从 uORB `dyt_command` 接收后转换为 DYT RS422 协议。

> 当前重要限制：固件尚未定义通用的“地面站 -> 飞控” `DYT_COMMAND` MAVLink 消息。地面站可直接发送 `DYT_GUIDANCE_COMMAND(12925)` 和专用点选命令 `DYT_TRACK_POINT_COMMAND(12935)`；其他导引头内部命令仍需通过远程 Shell 间接执行。

### 通用字段

| Field | Type | Description |
| --- | --- | --- |
| `command` | `uint8_t` | 命令号。 |
| `param_x` | `int16_t` | 角度命令时为 yaw×100 centidegree；跟踪点时为 X pixel。 |
| `param_y` | `int16_t` | 角度命令时为 pitch×100 centidegree；跟踪点时为 Y pixel。 |
| `param3` | `uint8_t` | 当前驱动未使用。 |
| `zoom_rate` | `int8_t` | 当前驱动未使用。 |
| `value` | `uint32_t` | 目标类型、目标 ID、图像模式等整数参数。 |
| `lat/lon` | `double` | 地理目标或本机纬经度，deg。 |
| `alt/rel_alt` | `float` | AMSL/相对高度，m。 |
| `roll_rad/pitch_rad/yaw_rad` | `float` | 本机姿态，rad。 |
| `airspeed_m_s/groundspeed_m_s` | `float` | 当前驱动的 flight-data 编码未使用这两个字段。 |

### 命令表

| Value | Name | Parameters | Current Driver Behavior |
| ---: | --- | --- | --- |
| 0 | `CMD_NONE` | - | 无操作。 |
| 1 | `CMD_AUTO_LOCK` | 可选 `param_x=-100` 由制导内部使用 | 进入跟踪模式，默认跟踪点 `(0,0)`。 |
| 2 | `CMD_STOP_TRACK` | - | 发送停止跟踪器命令并回零角度模式。 |
| 3 | `CMD_RETRIGGER` | - | 停止跟踪、开启辅助、重新进入跟踪模式。 |
| 4 | `CMD_NOFOLLOW` | - | 进入 LOCK 模式。 |
| 5 | `CMD_CENTER` | - | 进入 HOME/居中模式。 |
| 6 | `CMD_YAW_FOLLOW` | - | 进入 FOLLOW_ANGLE 模式。 |
| 7 | `CMD_ELECT_LOCK` | - | 已定义，但当前驱动 switch 未实现，发送后无动作。 |
| 8 | `CMD_ELECT_UNLOCK` | - | 已定义，但当前驱动 switch 未实现，发送后无动作。 |
| 9 | `CMD_LOCK_VIEW` | - | 进入 LOCK 模式。 |
| 10 | `CMD_CENTER_GIMBAL` | `param_x=yaw_cdeg`, `param_y=pitch_cdeg` | 发送角度并进入 FOLLOW_ANGLE。 |
| 11 | `CMD_SET_FRAME_ANGLE` | `param_x=yaw_cdeg`, `param_y=pitch_cdeg` | 框架角控制；跟踪模式保护期内会忽略。 |
| 12 | `CMD_SEARCH_RATE` | `param_x=yaw_boundary_cdeg`, `param_y=pitch_boundary_cdeg` | 进入扫描模式，当前参数实际为扫描边界，不是角速度。 |
| 13 | `CMD_SEND_OWNSHIP_STATE` | `lat/lon/alt/rel_alt/roll/pitch/yaw` | 向导引头发送本机飞行数据。 |
| 14 | `CMD_GEO_TRACK` | `lat/lon/alt` | 地理目标跟踪；保护期内会忽略。 |
| 15 | `CMD_GEO_TRACK_EXIT` | - | 退出地理跟踪，进入 LOCK。 |
| 16 | `CMD_SET_INERTIAL_ANGLE` | - | DYT 协议不支持，当前明确不执行。 |
| 17 | `CMD_TRACK_POINT` | `param_x=x_px`, `param_y=y_px` | 指定图像像素点跟踪。 |
| 18 | `CMD_AI_ENABLE` | - | 开启 AI。 |
| 19 | `CMD_AI_DISABLE` | - | 关闭 AI。 |
| 20 | `CMD_TARGET_TYPE` | `value=1..3` | 设置识别目标类型。 |
| 21 | `CMD_ASSIST_ENABLE` | - | 开启辅助识别/跟踪。 |
| 22 | `CMD_ASSIST_DISABLE` | - | 关闭辅助识别/跟踪。 |
| 23 | `CMD_TRACK_ID` | `value=target_id` | 选择跟踪目标 ID。 |
| 24 | `CMD_IMAGE_MODE` | `value=0..9` | 可见光/红外/画中画/布局/黑白彩色。 |
| 25 | `CMD_LASER_ON` | - | 激光电源开启。 |
| 26 | `CMD_LASER_CONTINUOUS` | - | 连续激光开启。 |
| 27 | `CMD_LASER_OFF` | - | 先关闭连续激光，再关闭激光电源。 |

### CMD_TARGET_TYPE.value

| Value | Name |
| ---: | --- |
| 1 | ALL |
| 2 | PERSON |
| 3 | VEHICLE |

### CMD_IMAGE_MODE.value

| Value | Name | Description |
| ---: | --- | --- |
| 0 | VIS | 可见光。 |
| 1 | IR | 红外。 |
| 2 | PIP1 | 画中画模式 1。 |
| 3 | PIP2 | 画中画模式 2。 |
| 4 | UP1 | 上下布局 1。 |
| 5 | UP2 | 上下布局 2。 |
| 6 | LEFT1 | 左右布局 1。 |
| 7 | LEFT2 | 左右布局 2。 |
| 8 | BW | 黑白显示。 |
| 9 | COLOR | 彩色显示。 |

## 7. 网捕与前视激光测距

### 7.1 输入消息：DISTANCE_SENSOR (132)

前视 SDM50 直接连接飞控串口，由 `sdm50` 驱动发布 uORB `distance_sensor`；飞控再通过标准 MAVLink `DISTANCE_SENSOR` 转发给地面站。

| Field Name | Type | Units | SDM50 Value/Meaning |
| --- | --- | --- | --- |
| `time_boot_ms` | `uint32_t` | ms | 样本时间。 |
| `min_distance` | `uint16_t` | cm | 5 cm。 |
| `max_distance` | `uint16_t` | cm | 5000 cm。 |
| `current_distance` | `uint16_t` | cm | 当前距离。 |
| `type` | `uint8_t` | enum | `MAV_DISTANCE_SENSOR_LASER=0`。 |
| `id` | `uint8_t` |  | MAVLink stream 中的 uORB instance 序号，不保证跨重启固定。 |
| `orientation` | `uint8_t` | enum | 前视 SDM50 为 `ROTATION_NONE=0`。 |
| `signal_quality` | `uint8_t` | % | 有效样本为 100；无效目标为 1。 |

地面站区分前视/下视激光时必须优先使用 `orientation`，不要硬编码 `id`。

### 7.2 自动网捕触发条件

以下条件同时满足才发送 `MAV_CMD_DO_GRIPPER` release：

1. DYT 目标当前可用且数据新鲜；
2. 本次末制导会话尚未触发网捕；
3. `DYTG_FIRE_EN=1`；
4. `DYTG_RNG_MIN >= 0.05 m` 且 `DYTG_RNG_MAX > DYTG_RNG_MIN`；
5. 使用的是 SDM50、LASER、前视 `orientation=0`；
6. 样本年龄不超过 200 ms，质量大于 0，距离在传感器量程内；
7. `current_distance` 位于闭区间 `[DYTG_RNG_MIN, DYTG_RNG_MAX]`。

### 7.3 手动网捕

- `DYTG_FIRE_AUX`：指定 AUX 上升沿触发；默认 -1 禁用。
- `DYTG_FIRE_BTN`：指定 QGC joystick button 上升沿触发；默认 -1 禁用。
- 手动触发绕过激光距离窗，但仍要求 DYT target usable 且本次会话未触发。

### 7.4 网捕反馈

地面站读取 `DYT_SYSTEM_STATUS`：

- `status_flags & 0x0100`：当前末制导会话已经发送过释放命令；
- `net_trigger_count`：本次开机累计发送次数。

这两个字段表示**飞控已发布触发指令**，不是网捕机构已物理动作。标准 gripper 执行链的 `COMMAND_ACK` 也不能替代独立的机构位置/限位反馈。

### 7.5 网捕后控制

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `DYTG_HOLD_EN` | 1 | bool | 释放后立即制动并转位置保持。 |
| `DYTG_STOP_D` | 0.5 | m | 目标水平停车距离。 |
| `DYTG_STOP_V` | 0.2 | m/s | 低于该速度转位置保持。 |
| `DYTG_STOP_ACC` | 4.0 | m/s² | 自适应制动最大加速度，同时受 `DYTG_MAXACC` 限制。 |
| `DYTG_NET_EN` | 1 | bool | 监听 gripper release 并启用短时减速。 |
| `DYTG_NET_MS` | 500 | ms | 减速时间窗。 |
| `DYTG_NET_SC` | 0.5 | ratio | 释放后的水平速度比例。 |
| `DYTG_NET_ACC` | 4.0 | m/s² | 反向减速度上限。 |

## 8. 下视激光测距降落

### 8.1 MT-06 接入

- MT-06 接在 GPS2 串口，115200 baud。
- 传感器通过 MAVLink 向飞控发送标准 `DISTANCE_SENSOR(132)`。
- 下视传感器必须设置 `orientation=MAV_SENSOR_ROTATION_PITCH_270=25`。
- `type=MAV_DISTANCE_SENSOR_LASER=0`。
- 飞控将接收到的 uORB distance sensor 再通过 MAVLink stream 发给地面站。
- 当前 `EKF2_RNG_CTRL=1`，允许 EKF 使用下视距离。

### 8.2 快速触地条件

快速触地不是地面站命令，而是 land detector 内部逻辑。必须同时满足：

1. `LNDMC_TD_EN=1`；
2. 飞机已解锁并处于飞行状态；
3. AUTO_LAND、AUTO_RTL 或 AUTO_MISSION 中存在 LAND setpoint；
4. 下视 LASER 样本新鲜、有效、质量非 0；
5. 本地高度、Home 高度和 EKF range fusion 有效；
6. 飞机正在下降，且 EKF `dist_bottom < 0.5 m`；
7. 高度不超过 `LNDMC_TD_ALT`；
8. 距离不超过 `LNDMC_TD_DIST`；
9. 至少 3 个不同样本，持续时间不短于 `LNDMC_TD_TIME`；
10. 距离跳变满足内部速率/幅度限制。

### 8.3 当前板级参数

| Parameter | Value | Units | Description |
| --- | ---: | --- | --- |
| `LNDMC_TD_EN` | 1 | bool | 开启快速触地。 |
| `LNDMC_TD_DIST` | 0.1 | m | 传感器到地面的触发距离。 |
| `LNDMC_TD_ALT` | 5.0 | m | 相对 Home 独立高度上限。 |
| `LNDMC_TD_MAX` | 5.0 | m | 快速触地使用的最大测距。 |
| `LNDMC_TD_TIME` | 0.04 | s | 最小连续确认时间，且至少 3 帧。 |
| `COM_DISARM_LAND` | 0.05 | s | landed 后自动上锁延迟。 |

### 8.4 地面站显示建议

- 按 `orientation=25` 显示“下视/降落激光距离”；
- 按 `orientation=0` 显示“前视/网捕距离”；
- `signal_quality=0/1` 显示无效，不能把距离值用于触地或网捕提示；
- 用 `EXTENDED_SYS_STATE.landed_state` 显示落地状态；
- 当前没有“fast touchdown 已触发”的专用 MAVLink 字段，只会产生 STATUSTEXT：`ToF fast touchdown: ...`。

## 9. 应急策略及地面站可见信息

## 9.1 通信中断应急 comm_emergency

### 状态机

| State | Trigger | Action |
| --- | --- | --- |
| IDLE | 已解锁、空中，且曾见过 GCS 后链路丢失 | 请求 AUTO_LOITER/Hold |
| HOLDING | 链路恢复，原用户意图为 Mission | 恢复 AUTO_MISSION |
| HOLDING | 等待超过 `CEM_WAIT` | `CEM_TO_ACT=0` 请求 RTL；`=1` 请求 LAND |
| HOLDING | 电量低于 `CEM_BAT_THR` | 剩余飞行时间足够安全 RTL 则 RTL，否则原地 LAND |
| COMMITTED | 已决定 RTL/LAND | 不再因链路恢复自动回 Mission |

### 参数

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `CEM_EN` | 1 | bool | 启用通信应急。 |
| `CEM_WAIT` | 30 | s | 断链 Hold 等待时间。 |
| `CEM_TO_ACT` | 0 | enum | 0=RTL，1=LAND。 |
| `CEM_BAT_THR` | 0.40 | norm | 低电量提前决策阈值。 |

### 地面站可见方式

- `HEARTBEAT.custom_mode`：AUTO_LOITER、AUTO_MISSION、AUTO_RTL、AUTO_LAND；
- `HEARTBEAT.system_status`：PX4 总体状态；
- `BATTERY_STATUS.battery_remaining/time_remaining`：应急决策输入；
- `EXTENDED_SYS_STATE.landed_state`：是否空中/降落/落地；
- `STATUSTEXT`：`communication emergency action: N`。

当前没有单独的 `COMM_EMERGENCY_STATUS` MAVLink 消息，地面站无法可靠显示 IDLE/HOLDING/COMMITTED、断链计时、RTL feasible 或 pending action。

## 9.2 GNSS 异常应急 gnss_emergency

### 触发和接管逻辑

1. 飞机已解锁且处于空中时，在任何飞行模式下监测 GNSS。
2. GPS 数据连续超过 3 秒未更新或搜星数变为 0 时，进入 Landing 状态；fix 降级和 jamming/spoofing 不触发本模块。
3. 飞控请求 `AUTO_LAND`；若宽松本地位置也不可用，PX4 自动退化为 `DESCEND`，GNSS 自行恢复不会中断降落。
4. 只有实体 RC 链路有效且飞手通过 RC 模式开关发出模式请求时，才释放应急降落。
5. RC 接管后保持 Released 状态；若 GPS 仍异常且 RC 再次丢失，则重新降落，GNSS 恢复后重新布防。

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `GEM_EN` | 1 | bool | 启用 GNSS 数据丢失/零搜星应急。 |

### 地面站可见方式

- `HEARTBEAT.custom_mode`：AUTO_LAND 或 RC 接管后的模式；
- 标准 GPS/估计器消息：GPS fix 和位置有效性；
- `EXTENDED_SYS_STATE`：下降/落地结果；
- `STATUSTEXT`：`GNSS emergency action: N`、`GNSS emergency released by RC mode change`。

当前没有单独的 `GNSS_EMERGENCY_STATUS` 消息，GPS 丢失判据、RC 接管状态和 pending action 不能通过一个稳定业务字段读取。

## 10. 标准 COMMAND_LONG / COMMAND_ACK 使用原则

标准 PX4 操作继续使用 MAVLink command microservice：

| MAV_CMD | ID | Purpose |
| --- | ---: | --- |
| `MAV_CMD_DO_SET_MODE` | 176 | 切换 Mission/Hold/RTL/Land 等模式。地面站优先使用 QGC/PX4 已有模式接口。 |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 20 | 请求 RTL。 |
| `MAV_CMD_NAV_LAND` | 21 | 请求降落。 |
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | 解锁/上锁。 |
| `MAV_CMD_DO_GRIPPER` | 211 | 网捕机构/夹爪动作；释放时 `param1=1`、`param2=0`。 |

地面站发送 `COMMAND_LONG` 后必须匹配 `COMMAND_ACK.command`、`target_system` 和 `target_component`，处理 `ACCEPTED/IN_PROGRESS/DENIED/FAILED/UNSUPPORTED`。不要用 STATUSTEXT 代替命令确认。

`DYT_GUIDANCE_COMMAND` 是本项目自定义状态机协议，不返回标准 `COMMAND_ACK`，而是通过 `DYT_SYSTEM_STATUS.command_sequence/command_result` 应答。

## 11. 地面站推荐状态页面

### 11.1 飞机和协同状态

- sysid、`vehicle_type`、格斗机/网捕机标签；
- group_id、leader/follower；
- cooperative 目标 ID、目标位置年龄；
- `guidance_phase` 0/1/2/3；
- `MIDCOURSE_ACTIVE`、`MIDCOURSE_TARGET_VALID`；
- 当前飞行模式和 armed/landed 状态。

### 11.2 导引头状态

- video_source：VIS1/VIS2/IR1/IR2；
- tracking_state、target_valid、target_fresh；
- tracking_algorithm；
- LOS X/Y、云台 roll/pitch/yaw；
- bbox、range、zoom；
- laser_on、motor_on、recording、follow_mode；
- gyro/servo/image-board fault；
- frame_dt、last_rx_age、parse_error_count。

### 11.3 制导控制

- 中制导按钮：发送 phase=2；
- 末制导按钮：发送 phase=3；
- request_id、PENDING/ACCEPTED/DENIED/FAILED；
- guidance_state、active_submode、lost_reason；
- controlling_vehicle；
- LOS、速度/加速度/yaw setpoint 曲线。

### 11.4 激光和网捕

- orientation=0：前视 SDM50 距离、质量和新鲜度；
- orientation=25：下视 MT-06 距离、质量和新鲜度；
- net_trigger_sent、net_trigger_count；
- 明确显示“命令已发送”，不要显示成“网捕成功”。

### 11.5 应急

- GCS link lost、GPS fix、估计器位置有效性；
- battery remaining/time remaining；
- 当前模式 Hold/RTL/Land/Descend；
- STATUSTEXT 告警历史。

## 12. 地面站接入示例流程

### 12.1 进入中制导

1. 生成 `request_id=N`。
2. 向目标 sysid 发送 `DYT_GUIDANCE_COMMAND{N, target_system, target_component, phase=2}`。
3. 等待 `DYT_SYSTEM_STATUS.command_sequence=N`。
4. `command_result=1` 显示“切换中”；`2` 显示“中制导已进入”；`3/4` 显示失败。
5. 同时确认 `guidance_phase=2`、`status_flags.MIDCOURSE_ACTIVE=1`。

### 12.2 进入末制导

1. 生成新的 `request_id=N+1`。
2. 发送 phase=3。
3. 等待对应 command_sequence/result。
4. `guidance_state=SEARCH_WAIT_LOCK` 表示已授权、尚未锁定；不能显示成已经视觉跟踪。
5. `guidance_state=TRACK_FOLLOW/TRACK_INTERCEPT` 且 `DYT_CONTROLS_VEHICLE=1` 才表示 DYT 正在控制飞机。

### 12.3 网捕提示

1. 监听前视 `DISTANCE_SENSOR.orientation=0`。
2. 显示距离窗提示，但飞控才是实际自动触发判定者。
3. 检测 `net_trigger_count` 增加或 `NET_TRIGGER_SENT` 从 0 变 1。
4. 记录触发时刻、距离、制导状态和目标锁定状态。

### 12.4 激光降落提示

1. 监听下视 `DISTANCE_SENSOR.orientation=25`。
2. 联合 HEARTBEAT 模式、EXTENDED_SYS_STATE 和 signal_quality 显示。
3. 不应仅凭 `current_distance <= LNDMC_TD_DIST` 在地面站宣布触地；飞控还有高度、下降、range fusion、多帧和跳变校验。

## 13. 当前接口限制和待补消息

以下能力在飞控内部已经存在，但当前地面站协议尚未完整暴露：

1. **DYT 全命令控制未接入 MAVLink**：`DytCommand.msg` 的 0–27 命令只能由飞控内部或 NSH shell 发布。
2. **dyt_command 未镜像到地面站**：无法看到飞控刚向导引头发送了哪条命令及参数。
3. **导引头命令发送结果不完整**：`DYT_STATUS_REPLY` 是原始吊舱回复，但没有统一的 request_id、命令号、发送成功/设备执行成功关联。
4. **网捕只有“命令已发送”状态**：缺少机构动作成功、限位或故障反馈。
5. **快速触地没有结构化状态**：只能从 `DISTANCE_SENSOR`、落地状态和 STATUSTEXT 推断。
6. **通信/GNSS 应急没有结构化状态**：无法直接显示状态机、原因、倒计时和 pending action。
7. **中/末制导完成确认仍是模块内部条件**：当前 `ACCEPTED` 不严格等价于 Commander 已确认 OFFBOARD。
8. **INITIAL 阶段判定较宽**：当前所有“已解锁但非中/末制导”的状态都会报 1，不严格等价于 Mission Takeoff。

如果地面站需要完整操作上述功能，建议后续预留并实现：

- `DYT_PAYLOAD_COMMAND`：GCS -> FC，带 request_id、目标 sys/component 和 DytCommand 全参数；
- `DYT_PAYLOAD_COMMAND_ACK`：FC -> GCS，区分接收、串口发送、吊舱回复；
- `DYT_AUXILIARY_STATUS`：网捕机构、前/下视激光、快速触地；
- `DYT_EMERGENCY_STATUS`：通信/GNSS 应急状态、原因、动作和倒计时。

在这些消息真正加入 XML、飞控收发代码并完成联调以前，地面站不得按“预留接口”发送或解析。

## 14. 联调检查表

- [ ] 地面站使用当前飞控同一份 `common.xml`，启用 MAVLink 2。
- [ ] 核对自定义消息 ID、payload length 和 CRC Extra。
- [ ] 多机命令明确填写 target_system，不使用广播误控全部飞机。
- [ ] `UAV_INFO.mavid>=100` 正确归并到真实飞机 ID。
- [ ] UAV_INFO 高度按 AMSL 解释，速度按 NED 解释。
- [ ] 中/末制导请求使用递增 request_id，并匹配 command_sequence。
- [ ] `SEARCH_WAIT_LOCK` 与“已锁定接管”分开显示。
- [ ] 前视/下视激光按 orientation 区分，不依赖 id。
- [ ] NaN、无效质量、超时样本不用于界面告警阈值。
- [ ] 网捕显示“触发命令已发送”，不等同于机构成功。
- [ ] 应急状态联合 HEARTBEAT、BATTERY_STATUS、EXTENDED_SYS_STATE、STATUSTEXT 显示。
- [ ] SITL 验证正常切换、拒绝、超时、重复包、断链、GPS 异常和人工接管。
- [ ] 实机先做无桨台架测试，再进行受控飞行测试。
