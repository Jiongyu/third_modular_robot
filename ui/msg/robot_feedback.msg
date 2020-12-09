# 时间戳
Header timeHeader
# 反馈关节位置数据 I1 T2 T3 T4 I5
float32[] feedbackPosData
# 反馈关节速度数据
float32[] feedbackVelData
# 反馈关节电流数据
float32[] feedbackCurrData
# 双爪抓夹情况(true:处于抓紧状态；false:处于松开状态),顺序：G0、G6
bool[] isGrasping
