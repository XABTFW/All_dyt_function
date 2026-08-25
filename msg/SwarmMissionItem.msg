# 集群任务航点消息 - 用于主机向从机广播任务航点
uint64 timestamp          # 时间戳 (微秒)

uint8 group_id            # 组号
uint8 leader_id           # 主机ID
uint32 mission_id         # 任务ID (用于检测任务更新)
uint16 total_count        # 任务总航点数
uint16 current_seq        # 当前航点序号

# 航点信息 (最多支持一次发送一个航点，通过多次发送完成全部航点同步)
uint16 seq                # 航点序号
uint16 nav_cmd            # 导航命令类型 (NAV_CMD_WAYPOINT等)
float64 lat               # 纬度 (度)
float64 lon               # 经度 (度)
float32 alt               # 高度 (米, AMSL)
float32 yaw               # 航向 (弧度, NED)
float32 acceptance_radius # 航点接受半径 (米)
float32 loiter_radius     # 盘旋半径 (米)
float32 time_inside       # 在航点停留时间 (秒)
bool autocontinue         # 是否自动继续下一航点

# 同步状态
uint8 sync_type           # 同步类型: 0=单个航点, 1=任务开始, 2=任务结束, 3=任务清除
uint8 SYNC_SINGLE = 0
uint8 SYNC_START = 1
uint8 SYNC_END = 2
uint8 SYNC_CLEAR = 3

# 转发标记
bool is_relay             # 是否为从机转发的消息（用于主机丢失后从机间同步航点）
