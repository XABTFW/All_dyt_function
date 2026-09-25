# DYT_POINTING_TARGET 导引头目指消息接口

## 用途与边界

`DYT_POINTING_TARGET` 是地面站/发送方发往飞控的独立 MAVLink 2 自定义消息，消息 ID 为 **12933**。它提供一个 WGS84 经纬度和 AMSL 高度位置，供 `dyt_guidance` 在中制导目指、以及视觉锁定丢失后的目指恢复中使用。它**只决定导引头看向哪里**，不写入 `follower_info`，不替代 `UAV_INFO`，也不直接改变飞机的跟随目标或航迹。

消息定义位于 `src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`。地面站和飞控必须使用包含该定义的同一版 dialect 生成 MAVLink 代码；消息 ID 大于 255，传输须使用 MAVLink 2。

## 字段

| 字段 | MAVLink 类型 | 单位与语义 | 发送要求 |
| --- | --- | --- | --- |
| `time_usec` | `uint64_t` | 发送方时间戳，微秒；飞控不要求其与自身时钟同步 | 非零；同一连续数据流严格递增 |
| `lat_int` | `int32_t` | WGS84 纬度，度 × 10⁷（`degE7`） | −900000000～900000000 |
| `lon_int` | `int32_t` | WGS84 经度，度 × 10⁷（`degE7`） | −1800000000～1800000000 |
| `alt` | `float` | 目标海拔高度，**AMSL，米**；不是相对起飞点高度，也不是离地高度 | 有限数值，不得为 NaN/Inf |

MAVLink payload 长度为 20 字节，CRC Extra 为 33。飞控拒收时间戳为 0、经纬度超出上述范围或高度非有限值的消息。接收成功后，飞控将纬经度转换为度并发布到 `dyt_pointing_target` uORB 主题；该主题的 `timestamp` 是飞控接收时间，`time_usec` 原样保留为发送方时间戳。

## 发送示例

以下示例使用由本项目 `common.xml` 生成的 MAVLink C 接口；它只展示消息打包，链路发送沿用地面站现有实现：

```c
const uint64_t time_usec = 1234567890123ULL; // 示例值；实际发送时持续递增
const int32_t lat_int = 399123456;   // 39.9123456°
const int32_t lon_int = 1163912345;  // 116.3912345°
const float alt_amsl_m = 123.5f;     // 海拔 123.5 m

mavlink_message_t msg;
mavlink_msg_dyt_pointing_target_pack(sender_sysid, sender_compid, &msg,
                                     time_usec, lat_int, lon_int, alt_amsl_m);
```

建议按目标位置更新持续发送，并让发送间隔明显短于 `DYTG_TGT_TO`；不要只发一次后期待飞控永久沿用旧目标。若发送端重启导致 `time_usec` 回退，在上一目标仍有效期间，新包不会替换旧包，旧包超时后才可接受新的时间序列。

