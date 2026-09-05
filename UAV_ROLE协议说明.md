# UAV_ROLE 飞机类型协议

## 身份与类型

- `MAV_SYS_ID`：PX4原生飞机ID，用于区分不同实体飞机，每架飞机必须唯一。
- `UAV_ROLE`：只负责飞控向地面站反馈飞机类型，不参与ID寻址、位置处理或飞行控制。

## MAVLink消息定义

| 项目 | 定义 |
| --- | --- |
| 消息名 | `UAV_ROLE` |
| 消息ID | `12932` |
| 方向 | 飞控 → 地面站 |
| 发送频率 | `1 Hz` |

| 字段 | 类型 | 定义 |
| --- | --- | --- |
| `mavid` | `uint32_t` | 当前飞机的 `MAV_SYS_ID` |
| `vehicle_type` | `uint8_t` | 当前飞机类型 |

## vehicle_type取值

| 值 | 飞机类型 |
| --- | --- |
| `1` | 撞击机 |
| `2` | 网捕机 |
| `3` | 测试目标机，地面站可以忽略 |

飞机端使用参数 `DYT_VEH_TYPE` 设置类型，取值与上表完全一致，默认值为 `2`。

## 地面站处理

地面站使用 MAVLink 帧头的 `message.sysid` 识别具体飞机，使用 `UAV_ROLE.vehicle_type` 判断该飞机是撞击机、网捕机还是测试目标机。

```text
message.sysid       → 哪一架飞机
UAV_ROLE.mavid      → 该飞机的 MAV_SYS_ID
UAV_ROLE.vehicle_type → 该飞机的任务类型
```
