#!/bin/zsh
echo "nv" | sudo -S ls /root
sshpass -p "nv" ssh 94622@192.168.100.31 "powershell; get-date" > /home/nv/massage.txt

file="/home/nv/massage.txt"

if [ ! -f "$file" ]; then
    echo "Error: File does not exist."
    exit 1
fi
converted_content=$(iconv -f GBK -t UTF-8 "$file" | sed 's/[^[:print:]]//g')
timestamp=$(echo "$converted_content" | grep -oP '(\d{4}年[0-9]{1,2}月[0-9]{1,2}日) (\d{1,2}:\d{2}:\d{2})')

if [ -z "$timestamp" ]; then
    echo "No timestamp found."
    exit 1
fi
year=$(echo "$timestamp" | grep -oP '(\d+)年' | grep -oP '\d+')
month=$(echo "$timestamp" | grep -oP '年(\d+)月' | grep -oP '\d+')
day=$(echo "$timestamp" | grep -oP '月(\d+)日' | grep -oP '\d+')

formatted_date=$(date -d "$year-$month-$day" '+%Y-%m-%d')
time_part=$(echo "$converted_content" | grep -oP '\d{1,2}:\d{2}:\d{2}') 

echo $formatted_date $time_part

echo "nv" | sudo -S date --set=$year-$month-$day
echo "nv" | sudo -S date --set=$time_part
