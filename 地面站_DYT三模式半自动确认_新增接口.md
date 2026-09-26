# DYT 三模式与半自动确认新增接口

## 1. 新增参数

参数名：`DYTG_MODE`

| 值 | 模式 | 行为 |
| ---: | --- | --- |
| 0 | 手动 | 飞控自动将 `DYTG_AUTO_EN` 设为 0。 |
| 1 | 半自动 | 飞控自动将 `DYTG_AUTO_EN` 设为 0。点选锁定后不进入末制导，等待地面站确认。 |
| 2 | 全自动 | 飞控自动将 `DYTG_AUTO_EN` 设为 1。 |

地面站通过标准 MAVLink `PARAM_SET` 设置：

```text
target_system=<飞控 sysid>
target_component=1
param_id="DYTG_MODE"
param_value=<整数 0 / 1 / 2 的 4 字节按位放入 float 字段；兼容直接赋值 0.0 / 1.0 / 2.0>
param_type=MAV_PARAM_TYPE_INT32
```

`DYTG_MODE` 是 `int32_t` 参数。使用 MAVLink `PARAM_SET` 时，飞控按字节读取
`param_value`：例如半自动的整数 `1` 对应原始位 `0x00000001`。飞控现在也兼容
直接写浮点 `1.0f`（原始位 `0x3F800000`），无论该包标记为 `INT32` 还是
`REAL32`，只对 `DYTG_MODE` 的 `0/1/2` 生效。若 MAVLink 库提供整数参数编码接口，应交给库编码；手工组包时可用 `memcpy` 将
`int32_t mode = 1` 的 4 字节复制到 `param_value`。发送后按同一规则解码
`PARAM_VALUE` 回包，并确认 `DYT_SYSTEM_STATUS.control_mode=1`、
`DYTG_AUTO_EN=0`。

飞控参数定义位置：

```text
src/modules/dyt_guidance/dyt_guidance_params.c
```

## 2. 新增反馈字段

`DYT_SYSTEM_STATUS(12926)` 新增两个 MAVLink 2 扩展字段：

| 字段 | 类型 | 值 |
| --- | --- | --- |
| `control_mode` | `uint8_t` | `0=手动`，`1=半自动`，`2=全自动` |
| `semi_auto_state` | `uint8_t` | `0=非半自动`，`1=空闲`，`2=已点选、等待锁定`，`3=已锁定、等待确认`，`4=末制导已激活` |

MAVLink XML 位置：

```text
src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml
```

地面站需用该 `common.xml` 重新生成 MAVLink 代码。`DYT_SYSTEM_STATUS` 的原有字段和 CRC Extra `0` 保持不变，新字段位于扩展区。

## 3. 半自动确认流程

1. 设置 `DYTG_MODE=1`。
2. 点选功能没有新增接口，继续使用当前 QGroundControl 已实现的链路。操作员点击视频画面后，地面站获得以左上角为原点的点击像素 `x/y`，然后在地面站内换算成以画面中心为原点的坐标：

```text
centered_x = x - image_width / 2
centered_y = y - image_height / 2
```

地面站通过 MAVLink `SERIAL_CONTROL(126)` 远程 Shell 发送：

```text
dyt_gimbal trackxy <centered_x> <centered_y>\n
```

飞控的 `trackxy` Shell 命令直接接收这两个中心坐标，并填入导引头跟踪模式 `0x06` 的坐标字段。地面站现有点选代码位置：

```text
src/Vehicle/Vehicle.cc
Vehicle::sendDytTrackPointCommand(...)
```

飞控现在支持两种点选锁定方式：

| 方式 | 地面站发送内容 | 坐标系 | 当前 QGroundControl |
| --- | --- | --- | --- |
| 结构化 MAVLink | `DYT_TRACK_POINT_COMMAND(12935)` | 左上角为原点的绝对像素，飞控负责转换成中心坐标 | 未使用 |
| 远程 Shell | `SERIAL_CONTROL(126)` 中发送 `dyt_gimbal trackxy <x> <y>` | 画面中心为原点的相对像素，地面站负责转换 | 正在使用 |

两种方式最终都在飞控内部生成 `CMD_TRACK_POINT`，半自动逻辑都能识别。同一次点选只能选择其中一种，不要同时发送 MAVLink 12935 和 Shell 命令，否则会重复下发锁定。本次半自动修改不要求地面站更换现有 Shell 点选链路。

3. 等待 `DYT_SYSTEM_STATUS.control_mode=1` 且 `semi_auto_state=3`。此时导引头保持目标锁定，点选操作不清除原有中制导请求，飞机继续由中制导控制；导引头末制导尚未接管，`status_flags bit3=0`。
4. 点击“确认开始”后发送已有 `DYT_GUIDANCE_COMMAND(12925)`：

```text
request_id=<非0递增序号>
target_system=<飞控 sysid>
target_component=1
phase=3
```

5. 通过 `DYT_SYSTEM_STATUS` 确认结果：

```text
command_sequence == request_id
command_result == 2
semi_auto_state == 4
guidance_phase == 3
```

只有这条 `phase=3` 确认消息被飞控接受后，才允许从中制导切换到末制导。点选锁定本身不得触发末制导。

如果在 `semi_auto_state=3` 之前发送确认，飞控返回 `command_result=3`，不进入末制导。

## 4. 中心锁定说明

导引头协议中的跟踪坐标以画面中心为原点，所以 `(0,0)` 表示“锁定画面中心”。点选任意其他位置时，发送的是换算后的非零中心坐标，并不是中心锁定。只有原有自动锁定 `CMD_AUTO_LOCK` 没有地面站点选坐标，所以使用 `(0,0)` 锁定画面中心目标。

半自动模式已禁止在点选后再发 `CMD_AUTO_LOCK(0,0)`，会保留现有 `trackxy` 点选坐标，避免把已点选目标改成画面中心锁定。
