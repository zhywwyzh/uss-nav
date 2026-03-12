int8 drone_id
# 1: baseFail (vins_odom too low or no odom)
# 2: basepreparing
# 3: baseReady
# 4: px4_take_off
# 5: command
# 6: px4_land
# 7: auto_hover
# 8: locatizationError (optical_flow)
# 9: crash (cmd ---> stopPropeller) 
# 10: drone_node_only
# 11: timeAlignFail
time start_time
int8 current_node_state
string debug_info