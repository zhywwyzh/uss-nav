service ntp stop
ntpdate -u 192.168.100.77
hwclock -w
service ntp start
