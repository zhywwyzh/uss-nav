# 扫描结果 (graph6 -> 上层): 区分流程结束与完整覆盖。

std_msgs/Header header
string session_id

uint8 COMPLETE=1
uint8 PARTIAL=2
uint8 ABORTED=3
uint8 status

uint32 planned_count
uint32 covered_count
uint32[] missing_waypoint_ids
string reason
