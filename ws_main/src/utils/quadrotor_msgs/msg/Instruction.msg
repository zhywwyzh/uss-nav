uint8 robot_id
uint8 instruction_type

uint16 target_obj_id
string command

#definations for instruction_type
uint8 TURN_OBJECT_NAV = 1
uint8 TURN_OBJECT_ID_NAV = 2
uint8 TURN_REGULAR_EXPLORATION = 3
uint8 TURN_DF_DEMO = 4