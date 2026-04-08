# 获取当前日期
current_date=$(date '+%Y-%m-%d')
# 获取当前时间
current_time=$(date '+%H:%M:%S')
# 组合日期和时间
full_datetime="$current_date $current_time"
# 设置系统日期和时间
xdotool key --repeat 1 alt+g
xdotool type "sudo timedatectl set-time "
xdotool type "\"$full_datetime\""
xdotool key Return
xdotool type 'nv'
xdotool key Return
# sleep 1
# xdotool type 'nv'
# xdotool key Return
