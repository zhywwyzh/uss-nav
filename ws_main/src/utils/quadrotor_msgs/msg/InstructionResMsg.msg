Header header
uint8 instruction_type
uint8 drone_id
bool is_succeed

# -- map merge and share map -- #
uint8[] tar_drone_id
uint8[] src_drone_id
uint8[] failed_drone_id

# -- instruction type defination-- #
uint8 TURN_GOAL = 1
uint8 TURN_EXPLORE = 2
uint8 TURN_PATROL = 3
uint8 TURN_HIT = 4
uint8 GO_HOME = 5
uint8 TURN_EGO_GOAL = 6
uint8 SHARE_MAP = 7
uint8 RESET_EXPLORE_AREA = 8
uint8 MAP_MERGE_REQUEST = 9
uint8 MAP_CIRCULATE = 10
