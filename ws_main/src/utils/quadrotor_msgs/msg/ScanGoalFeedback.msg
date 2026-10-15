# 扫描目标反馈 (ego -> graph6): 目标接受/改写/抵达/失败/取消的事件与终态。

std_msgs/Header header
uint8 drone_id

string session_id
uint32 request_id
uint32 waypoint_id
uint32 event_seq          # 每请求递增

uint8 ACCEPTED=1
uint8 ADJUSTED=2
uint8 REACHED=3
uint8 REJECTED=4
uint8 FAILED=5
uint8 CANCELLED=6
uint8 status

geometry_msgs/Point requested_goal
geometry_msgs/Point execution_goal
int32 candidate_index     # 未选定为 -1
uint64 map_seq
string reason
