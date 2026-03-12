Header header
uint32 prompt_id
uint8  option
uint8  prompt_type
string prompt
string answer

uint8 SEND_PROMPT = 0
uint8 SEND_ANSWER = 1

uint8 PROMPT_TYPE_ROOM_PREDICTION = 0
uint8 PROMPT_TYPE_EXPL_PREDICTION = 1
uint8 PROMPT_TYPE_MAP_FUSE_PREDIETION = 2
uint8 PROMPT_TYPE_TERMINATE_OBJ_ID = 3
uint8 PROMPT_TYPE_DF_DEMO = 4