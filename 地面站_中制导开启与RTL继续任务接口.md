# 地面站中制导开启与 RTL 继续任务接口

本文只说明地面站开启中制导、发送中制导目标，以及飞机处于 Return/RTL 时继续任务所需的 MAVLink 接口。以下消息必须使用本项目的 `common.xml` 生成，并通过 **MAVLink 2** 发送。

## 1. 接口总览

| 作用 | MAVLink 消息 | Message ID | 发送要求 |
|---|---|---:|---|
| 提供中制导目标位置 | `UAV_INFO` | `12921` | 任务期间持续发送，建议 `10 Hz` |
| 请求开启中制导/从 RTL 继续任务 | `DYT_GUIDANCE_COMMAND` | `12925` | 每次操作生成一个新的 `request_id` |
| 初始起飞并设置自动进入中制导的高度 | `MAV_CMD_NAV_TAKEOFF` | `22` | `param7/z` 填绝对海拔高度 `AMSL` |

`UAV_INFO` 只提供“飞向哪里”。正常手动开启和 RTL 恢复时，`DYT_GUIDANCE_COMMAND phase=2` 才表示“现在开始或恢复中制导”。初始起飞是唯一例外：飞控可根据本次起飞指令携带的高度自动产生一次中制导请求。仅发送目标位置不能替代开启请求；仅有开启请求但没有有效目标位置，飞机进入 Offboard 后只能保持位置，不能正常跟随目标。

## 2. 中制导目标位置：`UAV_INFO(12921)`

正常跟随靶机时填写：

| 字段 | 地面站填写值 |
|---|---|
| `point_source` | `1`，表示靶机实时坐标 |
| `mavid` | 靶机编号，必须与飞控 `cooperative_rendezvous start -t <编号>` 一致；当前 7Nano 配置为 `1` |
| `lat_int` | 目标纬度乘 `1e7`，`int32_t` |
| `lon_int` | 目标经度乘 `1e7`，`int32_t` |
| `rel_alt` | 目标的绝对海拔高度 `AMSL`，单位 `m`；字段名虽为 `rel_alt`，但这里禁止填写相对高度 |
| `vx` | 目标向北速度，单位 `m/s` |
| `vy` | 目标向东速度，单位 `m/s` |
| `vz` | 目标向下速度，单位 `m/s`，向上为负 |
| `yaw` | 目标航向，单位 `rad`；无有效值时填 `NaN` |
| `yaw_speed` | 目标航向角速度，单位 `rad/s`；无有效值时填 `NaN` |
| `land` | Bit0：目标正在降落；Bit1：目标已到达；没有时填 `0` |
| `group_id`、`is_leader` | 没有分组用途时均填 `0` |
| `target_system`、`target_component` | `point_source=1` 时均填 `0` |
| `lat`、`lon` | 同步填写浮点角度值，用于兼容旧飞控 |

发送频率使用 `10 Hz`。当前目标有效期由 `CRDZ_TGT_TOUT` 控制，默认 `2 s`；持续以 `10 Hz` 发送时不会因该超时失效。

## 3. 开启中制导：`DYT_GUIDANCE_COMMAND(12925)`

地面站发送以下字段：

| 字段 | 类型 | 地面站填写值 |
|---|---|---|
| `request_id` | `uint32_t` | 非零、单调递增的操作序号；每次新的开启/恢复操作必须换新值 |
| `target_system` | `uint8_t` | 接收飞机的 `MAV_SYS_ID` |
| `target_component` | `uint8_t` | `1`（`MAV_COMP_ID_AUTOPILOT1`）；也允许填 `0` |
| `phase` | `uint8_t` | 固定填 `2`，表示中制导 |

这不是 Shell 字符串，也不是 `MAV_CMD`。地面站必须按自定义 MAVLink 消息 `DYT_GUIDANCE_COMMAND` 编码发送。该消息载荷长度为 `7` 字节，CRC Extra 为 `169`。

发送规则：

1. 一次操作生成一个新的 `request_id`。
2. 在收到匹配的状态反馈前，可以重复发送同一个数据包；重发时保持同一个 `request_id`。
3. 收到 `DYT_SYSTEM_STATUS.command_sequence == request_id` 后停止重发。
4. 如果结果为拒绝或失败，排除原因后再次操作时必须递增 `request_id`，不能继续使用旧序号。

## 4. 初始起飞到指定高度自动开启一次中制导

地面站按标准 MAVLink 起飞接口发送 `MAV_CMD_NAV_TAKEOFF(22)`：

| 发送方式 | 高度字段 | 地面站填写值 |
|---|---|---|
| `COMMAND_LONG` | `param7` | 起飞目标绝对海拔高度 `AMSL`，单位 `m` |
| `COMMAND_INT` | `z`（飞控内部对应 `param7`） | 起飞目标绝对海拔高度 `AMSL`，单位 `m`；`coordinate_frame=MAV_FRAME_GLOBAL_INT(5)` |

`target_system` 填接收飞机的 `MAV_SYS_ID`，`target_component` 填 `1` 或 `0`。纬度、经度及其他起飞字段仍按标准 `MAV_CMD_NAV_TAKEOFF` 填写；禁止用 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` 发送相对高度。不新增自定义 MAVLink 字段，也不需要修改 XML。

飞控收到该外部起飞指令后执行：

1. 使用 `相对目标高度 = 起飞指令高度 AMSL - Home 高度 AMSL` 完成高度基准转换。
2. 使用本地 NED 高度计算当前相对起飞点高度：`Home.z - 当前 local_position.z`。
3. 飞机已解锁、已经离地、无 failsafe、位置和 Home 高度有效且 `DYTG_COOP_EN=1` 时，当前高度到达相对目标高度前 `0.5 m` 范围内，自动产生一次中制导请求。
4. 本次飞行的高度触发随即被消费；切换其他模式退出中制导后，高度仍满足也不会再次自动进入。
5. 网捕释放命令发出或飞控收到网捕释放事件时，立即清除当前自动中制导请求并永久禁止本次飞行再次由高度触发；因此后续减速悬停过程不会被高度条件打断。若要继续中制导，必须重新拨动中制导开关，或由地面站发送新的 `DYT_GUIDANCE_COMMAND(12925), phase=2`。

飞控只接收初次离地前收到的外部 `MAV_CMD_NAV_TAKEOFF` 作为该门限来源；离地后的重复起飞指令和网捕完成后的起飞指令不会重新建立高度触发。若起飞指令没有有效的 AMSL 高度，飞控不会自动开启中制导。

若到达高度门限时飞机的操作员意图已经是 RTL 或 Land，飞控只消费本次高度触发，不切换 Offboard；后续继续任务必须按第 6 节显式发送新的 `phase=2`。

自动高度触发没有 `DYT_GUIDANCE_COMMAND.request_id`，地面站应通过 `guidance_phase=2` 且 `status_flags` Bit6 为 `1` 确认已经进入中制导，不能用 `command_sequence` 判断这次自动切换。

## 5. 飞机正常状态下手动开始中制导

地面站按以下顺序处理：

1. 先以 `10 Hz` 持续发送有效的 `UAV_INFO(point_source=1)`。
2. 发送一次新的 `DYT_GUIDANCE_COMMAND`：`phase=2`。
3. 飞控先建立约 `1 s` 的 Offboard 数据预发送，然后请求进入 Offboard。
4. 地面站依据第 7 节的状态反馈确认是否真正进入中制导。
5. 中制导期间继续以 `10 Hz` 发送目标位置。

飞控接受开启请求的必要条件包括：飞机已解锁、本地水平和垂直位置有效、全局位置有效，并且 `DYTG_COOP_EN=1`。

## 6. Return/RTL 状态下继续任务

无论 RTL 是操作员主动选择、断链触发，还是其他原因触发，地面站主动恢复中制导时都使用同一流程：

1. 确认数据链已经恢复；仍处于通信断链时，飞控不会执行 Offboard 恢复。
2. 以 `10 Hz` 开始或继续发送有效的 `UAV_INFO(point_source=1)`。
3. 生成一个新的 `request_id`，发送：

   ```text
   DYT_GUIDANCE_COMMAND(12925)
   request_id=<新的非零递增序号>
   target_system=<飞机 MAV_SYS_ID>
   target_component=1
   phase=2
   ```

4. 请求被飞控接收后，`command_result` 先变为 `1`。如果当前 failsafe 尚未解除，请求会保留等待，但不会立即抢占安全保护。
5. failsafe 解除后，飞控预发送 Offboard 数据约 `1 s`，再从 RTL 请求切换到 Offboard 中制导。
6. 收到 `command_result=2`、`guidance_phase=2` 且 `status_flags` Bit6 为 `1`，才表示已经恢复中制导；此后继续以 `10 Hz` 发送目标位置。

地面站显式发送新的 `phase=2` 是操作员确认继续任务，因此该路径不受“累计水平航程小于 2 km”条件限制。但是它仍不能绕过仍在生效的 failsafe、未解锁或位置无效等安全条件。

## 7. 状态反馈：`DYT_SYSTEM_STATUS(12926)`

地面站用以下字段判断本次请求：

| 字段 | 判定方式 |
|---|---|
| `command_sequence` | 必须等于本次发送的 `request_id` |
| `command_phase` | 应等于 `2` |
| `command_result` | `0` 无命令；`1` 已接收、切换中；`2` 已接受并完成切换；`3` 拒绝；`4` 失败 |
| `gcs_phase_request` | 等于 `2` 表示当前保留的地面站中制导请求 |
| `guidance_phase` | 等于 `2` 表示飞控当前实际处于中制导阶段 |
| `status_flags` Bit6 | `1` 表示中制导控制已激活 |
| `status_flags` Bit7 | `1` 表示当前中制导目标有效 |

成功条件必须同时满足：

```text
command_sequence == 本次 request_id
command_result == 2
guidance_phase == 2
(status_flags & (1 << 6)) != 0
```

`DYT_SYSTEM_STATUS` 由 `DYT_TELEMETRY` 数据流上报，当前默认频率为 `10 Hz`；定义的最大载荷长度为 `93` 字节、生成头文件中的 `MIN_LEN` 为 `91`，CRC Extra 为 `0`。但 MAVLink 2 会自动删除载荷末尾连续的零字节，因此实际线上载荷可能短于 `91` 字节（例如后续状态字段为零时，成功应答可能是 `87` 字节）。地面站必须使用本项目 `common.xml` 生成的 MAVLink 2 解码器，由解码器自动为省略字段补零；禁止按固定的 `91` 或 `93` 字节长度拒收状态帧。

## 8. 断链 RTL 的原有“两公里自动恢复”规则

这一规则只用于断链触发 RTL 后的数据链自动恢复，不等同于地面站显式发送 `phase=2`：

- 断链超时并已经进入 RTL 后，如果数据链恢复、累计水平航程小于 `2000 m`，飞控会自动尝试恢复断链前记录的 Mission 或 Offboard 状态。
- 累计水平航程达到或超过 `2000 m` 时，不执行上述自动恢复，继续 RTL。
- 地面站不发送新的 `phase=2`，只能阻止“地面站主动恢复”路径，不能取消飞控原有的两公里内自动恢复判断。
- 如果断链前记录的是 Offboard，中制导自动恢复到 Offboard 后没有新鲜的 `UAV_INFO`，飞机会保持当前位置，不会继续跟随旧目标；停止发送目标数据可以阻止继续跟飞，但不保证飞机仍保持在 RTL 模式。
- 如果需要在任何 RTL 下明确继续中制导，不依赖自动恢复结果，按第 6 节发送新的 `DYT_GUIDANCE_COMMAND phase=2`。

## 9. 地面站不得使用的替代方式

- 不要用 `SET_POSITION_TARGET_GLOBAL_INT` 代替中制导目标数据；本项目中制导目标使用 `UAV_INFO`。
- 不要只发送 `UAV_INFO` 并期待飞机从 RTL 切回中制导。
- 不要发送 Shell 命令开启中制导；开启和 RTL 恢复都使用 `DYT_GUIDANCE_COMMAND(12925)`。
- 不要把 `phase=3` 当成中制导开启；`phase=3` 是末制导请求。
