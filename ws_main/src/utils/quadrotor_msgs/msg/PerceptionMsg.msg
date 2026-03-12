Header header
# -------------------
uint16 from_drone_id
uint16 to_drone_id
uint8 req_or_resp
uint8 msg_type
MultiPoseGraph posegraph_msg
HgridMsg hgrid_msg
FrontierMsg ftr_msg

uint8 DATA_NEED_MAP_MERGE = 1
uint8 DATA_NEED_MAP_RESET = 2
uint8 RESPONSE_MSG_FALG = 2
uint8 REQUESET_MSG_FLAG = 1