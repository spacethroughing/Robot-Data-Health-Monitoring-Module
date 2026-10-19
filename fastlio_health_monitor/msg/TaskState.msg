# ROS接口数据流：控制器/任务生产者 -> 消息或服务 -> 健康生命周期/规划适配器。
# 任务状态源时间；必须与规划路径stamp处于同一已知时钟域。
std_msgs/Header header
# 任务目标标识；活动目标不能为空，目标变化将清除旧路径就绪证据。
string goal_id
# 是否存在需要生成路径的目标；false表示IDLE，不要求持续产生路径。
bool active
# 可读任务状态，辅助诊断记录。
string state
