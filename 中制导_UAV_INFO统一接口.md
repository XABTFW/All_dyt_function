# 中制导 UAV_INFO 统一接口（地面站开发）

## 消息

- MAVLink 2 消息：`UAV_INFO`
- Message ID：`12921`
- CRC Extra：`100`
- 定义位置：`src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`
- 新版载荷长度：`56` 字节，兼容旧版 `45` 字节载荷

## 数据来源标记 `point_source`

| 值 | 数据类型 | 飞控处理 |
|---:|---|---|
| `1` | 靶机实时坐标 | 按 `mavid` 与飞控启动参数 `-t` 选择目标，执行偏移、滤波和 `CRDZ_HIST` 延迟跟随 |
| `2` | 地面站绕飞航点 | 作为固定 Offboard 位置目标，有效期由 `CRDZ_GCS_TOUT` 控制 |
| `3` | 地面站历史点 | 仅在 `CRDZ_HIST_EN=0` 时接受并优先跟随；超时后恢复使用实时目标坐标 |

`point_source` 是数据来源标志。新版地面站必须根据数据类型填写：`1` 表示目标机实时坐标，`2` 表示地面站绕飞点，`3` 表示地面站提供的历史点（仅在 `CRDZ_HIST_EN=0` 时使用）。新版地面站禁止将该字段设置为 `0`，因为 `0` 仅用于兼容现有旧版地面站的 45 字节 `UAV_INFO`，表示数据来源未显式声明，由飞控按照旧规则解析。

## 地面站必填字段

| 字段 | 类型 | 靶机坐标 `point_source=1` | 绕飞航点 `point_source=2` |
|---|---|---|---|
| `mavid` | `uint32_t` | 靶机编号，必须与飞控 `cooperative_rendezvous start -t <编号>` 一致；当前7-Nano启动配置为 `1` | `0` |
| `group_id` | `uint32_t` | 组号，无分组时为 `0` | `0` |
| `is_leader` | `uint8_t` | `0` | `0` |
| `point_source` | `uint8_t` | `1` | `2` |
| `target_system` | `uint8_t` | `0` | 接收飞机 `MAV_SYS_ID`，`0` 为广播 |
| `target_component` | `uint8_t` | `0` | `MAV_COMP_ID_AUTOPILOT1 (1)`，`0` 为任意组件 |
| `lat_int` | `int32_t` | 靶机纬度，角度乘 `1e7` | 航点纬度，角度乘 `1e7` |
| `lon_int` | `int32_t` | 靶机经度，角度乘 `1e7` | 航点经度，角度乘 `1e7` |
| `lat` / `lon` | `float` | 同步填入角度值，仅用于兼容旧飞控 | 同步填入角度值，仅用于兼容旧飞控 |
| `rel_alt` | `float` | 靶机绝对海拔 AMSL，单位 `m` | 航点绝对海拔 AMSL，单位 `m` |
| `vx/vy/vz` | `float` | NED速度，单位 `m/s`，`vx`北、`vy`东、`vz`向下为正 | 全部填 `0` |
| `yaw/yaw_speed` | `float` | 单位分别为 `rad`、`rad/s`，无效时填 `NaN` | 填 `NaN` |
| `land` | `uint32_t` | Bit 0：正在降落；Bit 1：已到达目标 | `0` |

`point_source=3` 的字段要求：

- `mavid` 必须与靶机编号相同，当前7-Nano为 `1`。
- `lat_int/lon_int/rel_alt` 填写历史点经纬度和绝对海拔 `AMSL`。
- `vx/vy/vz` 填写历史点对应的 NED 速度；没有速度时全部填 `0`。
- 其余字段按 `point_source=1` 填写。

## 高度基准

| 数据 | 高度字段 | 地面站应发送的高度 |
|---|---|---|
| 靶机实时坐标 `point_source=1` | `rel_alt` | 绝对海拔高度 `AMSL`，单位 `m` |
| 绕飞航点 `point_source=2` | `rel_alt` | 绝对海拔高度 `AMSL`，单位 `m` |
| 旧版45字节 `UAV_INFO point_source=0` | `rel_alt` | 绝对海拔高度 `AMSL`，单位 `m` |
| 历史点 `point_source=3` | `rel_alt` | `CRDZ_HIST_EN=0` 时由地面站发送绝对海拔 `AMSL`，单位 `m` |

- 统一后的 `UAV_INFO` 中没有需要发送相对高度的功能，不能把相对起飞点或相对Home点高度填入 `rel_alt`。

## 起飞高度

起飞接口与中制导 `UAV_INFO` 无关，高度基准如下：

| 起飞数据 | 高度定义 |
|---|---|
| `MAV_CMD_NAV_TAKEOFF.param7` | 绝对海拔高度 `AMSL`，单位 `m` |
| `MIS_TAKEOFF_ALT` | 未显式指定有效起飞高度时使用的默认相对爬升高度，相对起飞时飞机当前高度，单位 `m` |

- 地面站直接发送 `MAV_CMD_NAV_TAKEOFF` 时，`param7` 必须填写目标绝对海拔，不能直接填写期望爬升高度。
- 例如起飞点海拔为 `120 m`、期望爬升 `30 m`，则 `MAV_CMD_NAV_TAKEOFF.param7=150 m`；若不显式下发有效高度，可以设置 `MIS_TAKEOFF_ALT=30 m`。

## 发送要求

- 靶机坐标需持续发送，中断超过 `CRDZ_TGT_TOUT` 后飞控不再使用该目标。
- `CRDZ_HIST_EN=1` 时忽略地面站的 `point_source=3`，飞控使用 `point_source=1` 自行生成延迟历史点。
- `CRDZ_HIST_EN=0` 时，持续接收到的 `point_source=3` 优先于 `point_source=1`；其中断超过 `CRDZ_TGT_TOUT` 后恢复跟随仍然有效的 `point_source=1`。
- 绕飞航点需持续发送，发送周期必须小于 `CRDZ_GCS_TOUT`；默认 `0.6 s`时建议不低于 `5 Hz`。
- 有效的绕飞航点优先于靶机坐标；绕飞航点超时后，飞控自动恢复使用仍然有效的靶机坐标。
- 中制导绕飞航点只接受 `UAV_INFO point_source=2`。`SET_POSITION_TARGET_GLOBAL_INT` 和
  `SET_POSITION_TARGET_LOCAL_NED` 都不会转换为中制导绕飞航点。
- 上述两个标准消息仍保留原生 PX4 Offboard 位置控制功能，不属于本中制导接口。
- 进入中制导后，地面站不得继续发送 `SET_POSITION_TARGET_GLOBAL_INT` 或
  `SET_POSITION_TARGET_LOCAL_NED`；它们仍可通过 PX4 原生 Offboard 通道写入轨迹设定值，可能与中制导输出竞争。
