# 地面站通过 MAVLink 控制导引头说明

## 1. 文档范围


- 在地面站 MAVLink Console 中点击“发送”并执行 `dyt_gimbal ...` 时，使用的是 MAVLink/PX4 自带的远程 Shell 机制 `SERIAL_CONTROL(126)`。
- 飞控收到文本后，在机上执行 `dyt_gimbal` 命令，然后再通过 uORB 和 Tweety V2.0.9.6 RS422 协议控制导引头。

当前代码中存在两种通过 MAVLink 连接发起导引头操作的方式：

1. **自带远程终端方式**：标准 MAVLink `SERIAL_CONTROL(126)` 只负责把 `dyt_gimbal` NSH 命令文本送到飞控执行；光源、激光、AI、OSD、云台角度和跟踪等操作当前都使用这种间接方式。
2. **专用结构化消息方式**：自定义 MAVLink `DYT_TRACK_POINT_COMMAND(12935)` 用于点选并锁定目标，`DYT_GUIDANCE_COMMAND(12925)` 用于确认开始制导，飞控分别通过 `DYT_TRACK_POINT_ACK(12936)` 和 `DYT_SYSTEM_STATUS(12926)` 反馈。

> 当前没有专用的 `DYT_PAYLOAD_COMMAND` 自定义 MAVLink 消息。除像素跟踪外，其他导引头操作都是“MAVLink 自带远程 Shell + 飞控本地 `dyt_gimbal` 命令”的间接控制，不能称为专用导引头 MAVLink 控制协议。

### 1.1 本文档使用的 MAVLink 消息总表

| ID | 消息 | 方向 | Payload | CRC Extra | 用途 |
| ---: | --- | --- | ---: | ---: | --- |
| 126 | `SERIAL_CONTROL` | GCS <-> FC | 81 B（包含 MAVLink 2 扩展字段） | 220 | 发送 `dyt_gimbal` Shell 文本及接收 Shell 输出。 |
| 12925 | `DYT_GUIDANCE_COMMAND` | GCS -> FC | 7 B | 169 | 半自动锁定后确认开始末制导。 |
| 12926 | `DYT_SYSTEM_STATUS` | FC -> GCS | 91–93 B | 0 | 反馈控制模式、半自动状态和制导命令结果。 |
| 12927 | `DYT_TARGET_STATUS` | FC -> GCS | 83 B | 157 | 导引头跟踪、光源、云台、LOS 和激光状态。 |
| 12928 | `DYT_STATUS_REPLY` | FC -> GCS | 29 B | 227 | 导引头原始状态/回复转发。 |
| 12935 | `DYT_TRACK_POINT_COMMAND` | GCS -> FC | 14 B | 183 | 下发图像像素跟踪点。 |
| 12936 | `DYT_TRACK_POINT_ACK` | FC -> GCS | 11 B | 26 | 像素跟踪请求的飞控校验和发布结果。 |

## 2. 总体数据链路

### 2.1 MAVLink 自带远程 Shell 链路

```text
地面站
  -> MAVLink SERIAL_CONTROL(126)
  -> PX4 MAVLink Shell
  -> 在飞控上执行 dyt_gimbal ...
  -> uORB dyt_command
  -> dyt_gimbal 驱动
  -> Tweety V2.0.9.6 RS422
  -> 导引头
```

### 2.2 像素点跟踪链路

```text
地面站
  -> MAVLink DYT_TRACK_POINT_COMMAND(12935)
  -> PX4 mavlink_receiver
  -> uORB dyt_command/CMD_TRACK_POINT
  -> dyt_gimbal 驱动
  -> Tweety V2.0.9.6 RS422
  -> 导引头

飞控
  -> MAVLink DYT_TRACK_POINT_ACK(12936)
  -> 地面站
```

## 3. 通过自带 SERIAL_CONTROL(126) 执行飞控本地命令

`SERIAL_CONTROL` 的基础消息是标准 MAVLink 消息。本项目的 `common.xml` 包含 MAVLink 2 `target_system/target_component` 扩展字段；地面站现有 MAVLink 库如果没有这两个参数，必须用本项目 `common.xml` 重新生成，否则只能发出目标 ID 为 0 的广播 Shell 命令。


## 4. 通过远程 Shell 间接执行光源/画面模式切换

地面站将下列完整文本放入 `SERIAL_CONTROL.data`，并在末尾加换行符 `\n`。

| 地面站操作 | `SERIAL_CONTROL.data` 文本 | 导引头内部图像模式值 |
| --- | --- | ---: |
| 可见光 | `dyt_gimbal imagemode vis\n` | 0 |
| 红外 | `dyt_gimbal imagemode ir\n` | 1 |
| 画中画 1 | `dyt_gimbal imagemode pip1\n` | 2 |
| 画中画 2 | `dyt_gimbal imagemode pip2\n` | 3 |
| 上下布局 1 | `dyt_gimbal imagemode up1\n` | 4 |
| 上下布局 2 | `dyt_gimbal imagemode up2\n` | 5 |
| 左右布局 1 | `dyt_gimbal imagemode left1\n` | 6 |
| 左右布局 2 | `dyt_gimbal imagemode left2\n` | 7 |
| 黑白 | `dyt_gimbal imagemode bw\n` | 8 |
| 彩色 | `dyt_gimbal imagemode color\n` | 9 |

例如，切换到红外时发送的实际 ASCII 字节为：

```text
64 79 74 5F 67 69 6D 62 61 6C 20 69 6D 61 67 65 6D 6F 64 65 20 69 72 0A
 d  y  t  _  g  i  m  b  a  l     i  m  a  g  e  m  o  d  e     i  r \n
```

## 5. 可通过远程 Shell 间接执行的全部导引头命令

| 功能 | `SERIAL_CONTROL.data` 文本 | 参数和注意事项 |
| --- | --- | --- |
| 查看驱动状态 | `dyt_gimbal status\n` | 只读，不控制导引头。 |
| 画面中心自动锁定 | `dyt_gimbal autolock\n` | 进入 TRACK，默认跟踪点 `(0,0)`。 |
| 停止跟踪 | `dyt_gimbal stoptrk\n` | 停止跟踪后回到零角度随动。 |
| 云台回中 | `dyt_gimbal center\n` | 进入 HOME 模式。 |
| 开激光电源 | `dyt_gimbal laser on\n` | 只开电源。 |
| 开连续激光测距 | `dyt_gimbal laser continuous\n` | 发送连续测距开启命令。 |
| 关闭激光 | `dyt_gimbal laser off\n` | 先关连续测距，再关激光电源。 |
| 开/关 OSD | `dyt_gimbal osd on\n` / `dyt_gimbal osd off\n` | 控制导引头字符显示。 |
| 设置云台角度 | `dyt_gimbal angle <yaw_deg> <pitch_deg>\n` | yaw 度，pitch 度；驱动限制 yaw `[-180,180]`、pitch `[-90,90]`。 |
| 指定跟踪坐标 | `dyt_gimbal trackxy <x> <y>\n` | 有符号 16 位坐标；驱动说明中 `(0,0)` 为画面中心。 |
| 按 AI 目标 ID 跟踪 | `dyt_gimbal trackid <id>\n` | `id` 范围 `0..UINT32_MAX`。 |
| 开/关 AI 检测 | `dyt_gimbal ai on\n` / `dyt_gimbal ai off\n` |  |
| 开/关智能辅助跟踪 | `dyt_gimbal assist on\n` / `dyt_gimbal assist off\n` |  |
| 检测全部目标 | `dyt_gimbal target all\n` | 内部值 1。 |
| 只检测人员 | `dyt_gimbal target person\n` | 内部值 2。 |
| 只检测车辆 | `dyt_gimbal target vehicle\n` | 内部值 3。 |
| 切换光源/画面模式 | `dyt_gimbal imagemode <mode>\n` | `mode` 参见第 4 节。 |

## 6. 专用像素跟踪 MAVLink

### 6.1 三种控制模式

地面站通过标准 MAVLink `PARAM_SET` 设置参数 `DYTG_MODE`：

```text
target_system=<飞控 sysid>
target_component=1
param_id="DYTG_MODE"
param_value=0.0 / 1.0 / 2.0
param_type=MAV_PARAM_TYPE_INT32
```

| `DYTG_MODE` | 模式 | 飞控行为 |
| ---: | --- | --- |
| 0 | 手动 | 自动使 `DYTG_AUTO_EN=0`，不根据识别结果自动进入制导。 |
| 1 | 半自动 | 自动使 `DYTG_AUTO_EN=0`；点选后只保持导引头目标锁定，不清除现有中制导，收到确认后才由中制导切换到末制导。 |
| 2 | 全自动 | 自动使 `DYTG_AUTO_EN=1`，使用稳定识别和自动锁定逻辑。 |

`DYTG_AUTO_EN` 现在是兼容镜像参数，地面站不要再直接写它。

### 6.2 半自动地面站流程

1. 用 `PARAM_SET` 设置 `DYTG_MODE=1`，等待参数回传确认。
2. 继续使用现有 `DYT_TRACK_POINT_COMMAND(12935)` 发送点选坐标，不需要新的点选接口。
3. `DYT_TRACK_POINT_ACK.result == MAV_RESULT_ACCEPTED` 只表示飞控已接收点选命令。
4. 监听 `DYT_SYSTEM_STATUS`；当 `control_mode=1` 且 `semi_auto_state=3` 时，表示导引头已锁定并等待确认。此时点选不清除原有中制导请求，飞机仍按中制导目标位置飞行；`status_flags bit3=0` 表示末制导尚未接管。
5. 操作员点击“确认开始”后，发送 `DYT_GUIDANCE_COMMAND(12925)`：`request_id`为非 0 递增序号，`target_system`为目标飞控 sysid，`target_component=1`，`phase=3`。
6. 等待 `DYT_SYSTEM_STATUS.command_sequence == request_id`；`command_result=1` 表示正在切换，`command_result=2` 表示已进入末制导。此时 `semi_auto_state=4`，`guidance_phase=3`。

点选锁定本身不会进入末制导；只有收到并接受上述 `DYT_GUIDANCE_COMMAND(12925), phase=3` 后才从中制导切换到末制导。

如果导引头尚未真正锁定就发送确认，飞控返回 `command_result=3` 并且不进入制导。

### 6.3 DYT_TRACK_POINT_COMMAND(12935)

方向：`GCS -> FC`。

| 字段 | 类型 | 地面站填写要求 |
| --- | --- | --- |
| `request_id` | `uint32_t` | 必须非 0；每个新命令递增。 |
| `x_px` | `uint16_t` | XML 定义为从图像左边开始的像素坐标。 |
| `y_px` | `uint16_t` | XML 定义为从图像上边开始的像素坐标。 |
| `image_width_px` | `uint16_t` | 必须大于 0，且 `x_px < image_width_px`。 |
| `image_height_px` | `uint16_t` | 必须大于 0，且 `y_px < image_height_px`。 |
| `target_system` | `uint8_t` | 目标飞机 ID；多机环境禁止使用 0。 |
| `target_component` | `uint8_t` | 通常填 1。 |

地面站必须使用本项目修改后的 `common.xml` 重新生成 MAVLink 库，才能编解码消息 12935/12936。

pymavlink 调用形式：

```python
request_id += 1
master.mav.dyt_track_point_command_send(
    request_id,
    x_px,
    y_px,
    image_width_px,
    image_height_px,
    target_system,
    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
)
```

### 6.4 DYT_TRACK_POINT_ACK(12936)

方向：`FC -> GCS`。

| `result` | 含义 |
| ---: | --- |
| `MAV_RESULT_ACCEPTED` | 飞控完成参数校验并成功发布了内部 `dyt_command`。 |
| `MAV_RESULT_DENIED` | `request_id`、尺寸、坐标或重复请求不符合要求。 |
| `MAV_RESULT_FAILED` | 飞控内部 `dyt_command` 发布失败。 |

`ACCEPTED` 只表示飞控接受并发布了命令，不表示导引头已经锁定目标。


### 7.DYT_TARGET_STATUS(12927)

方向：`FC -> GCS`。主要用于确认：

- `video_source`：`0=VIS_1`、`1=VIS_2`、`2=IR_1`、`3=IR_2`；
- `tracking_state`：`0=SEARCH`、`1=LOCKED`、`2=TIMEOUT`、`3=ERROR`；
- `target_flags bit0`：目标数据有效；
- `target_flags bit4`：导引头电机开启；
- `target_flags bit5`：随动模式；
- `target_flags bit6`：导引头激光数据有效；
- `range_m`：导引头自身激光测距，不是前视 SDM50；
- `los_x_rad/los_y_rad`：水平/垂直视线误差。

判定导引头真正锁定时，至少同时满足：

```text
tracking_state == 1
target_flags & 0x0001 != 0
```
