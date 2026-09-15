# DYT 导引头状态上报接口

## MAVLink 消息

地面站接收 `DYT_TARGET_STATUS`，消息 ID `12927`，方向 `FC -> GCS`，MAVLink 2 Payload 长度 `83～98` 字节，CRC Extra `157`，默认上报频率 `10 Hz`。

地面站使用以下 XML 生成 MAVLink 代码：

```text
src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml
```

## 字段

| 信息 | MAVLink 字段 | 类型/单位 | 值 |
| --- | --- | --- | --- |
| 伺服状态 | `servo_status` | `uint8_t` | `0=未知`，`1=伺服帧正常`，`2=伺服帧超时`，`3=伺服数据错误` |
| 可见光/红外源 | `video_source` | `uint8_t` | 导引头原始值：`1=可见光`，`2=红外` |
| 跟踪状态 | `tracking_state` | `uint8_t` | `0=搜索/未锁定`，`1=已锁定`，`2=伺服帧超时`，`3=伺服数据错误` |
| 跟踪状态原始值 | `status2` | `uint8_t` | 导引头伺服状态帧第 32 字节原始值 |
| 脱靶量 | `miss_x_px` | `int16_t/pixel` | 方位脱靶量，导引头原始大端有符号值 |
| 脱靶量 | `miss_y_px` | `int16_t/pixel` | 俯仰脱靶量，导引头原始大端有符号值 |
| 云台横滚角 | `gimbal_roll_rad` | `float/rad` | 导引头横滚角度转换为弧度 |
| 云台俯仰角 | `gimbal_pitch_rad` | `float/rad` | 导引头俯仰角度转换为弧度 |
| 云台方位角 | `gimbal_yaw_rad` | `float/rad` | 导引头方位角度转换为弧度 |
| 倍率 | `zoom_ratio` | `float` | 当前光源状态帧的倍率原始值乘 `0.1`；无有效状态时为 `NaN` |
| 跟踪模式 | `servo_mode` | `uint8_t` | 见“伺服模式值” |
| 系统状态 | `self_test_raw` | `uint8_t` | 伺服状态帧“自检 1”原始值；厂家协议未定义枚举含义 |
| 软件版本有效 | `software_version_valid` | `uint8_t` | `0=尚未收到版本回复`，`1=版本有效` |
| 软件版本号 | `tracker_software_version` | `char[8]` | 跟踪器 `0x35` 回复的 8 个原始字符，不保证以 `\0` 结尾 |
| 激光距离 | `range_m` | `float/m` | 最近 `500 ms` 内的有效激光距离；无有效距离时为 `NaN` |
| 激光距离有效 | `target_flags bit6` | `uint16_t bitmask` | `1=range_m 有效`，`0=无有效距离` |

`tracking_state=1` 的飞控判定为：`servo_mode==0x06`，并且 `status2==DYT_TRK_VAL`；`DYT_TRK_VAL` 默认值为 `1`。

## 伺服模式值

| `servo_mode` | 含义 |
| ---: | --- |
| `0x00` | 关伺服 |
| `0x01` | 手动 |
| `0x02` | 收藏 |
| `0x03` | 随动 |
| `0x04` | 锁定（位置当前随动） |
| `0x05` | 扫描 |
| `0x06` | 跟踪 |
| `0x07` | 地理随动 |
| `0x08` | FPV |
| `0x09` | 稳像手动（横滚稳像） |
| `0x0A` | 开环模式 |
| `0xAA` | 使能 |

## 软件版本格式

飞控连接导引头串口后自动发送跟踪器软件版本读取命令 `0x35`，最多发送三次。收到回复后，`tracker_software_version` 按协议格式显示为：

```text
ab.c.d-ef.g.h
```

例如 8 个字符 `S100A100` 显示为 `S1.0.0-A1.0.0`。

## 独立激光距离消息

每次收到激光测距帧，飞控同时发送 `SDM50_STATUS`，消息 ID `12929`：

| 字段 | 类型/单位 | 值 |
| --- | --- | --- |
| `distance_m` | `float/m` | 本次激光距离 |
| `status` | `uint8_t` | `0=无效`，`1=有效` |
| `time_sample_ms` | `uint32_t/ms` | 本次激光帧接收时间 |

## 飞控实现位置

```text
msg/DytTarget.msg
src/drivers/dyt_gimbal/dyt_gimbal.cpp
src/modules/mavlink/streams/DYT_TELEMETRY.hpp
src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml
```
