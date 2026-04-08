#!/bin/bash

# 本地文件路径
LOCAL_FILE="/home/nv/multi_tracking_real/src/realflight_modules/FAST_LIO/PCD/origin.pcd"

# 远程计算机用户名
REMOTE_USER="nv"

# YAML 文件路径
YAML_FILE="/home/nv/multi_tracking_real/src/planner/swarm_bridge/config/ip.yaml"

# 远程目录路径
REMOTE_PATH="/home/nv/multi_tracking_real/src/realflight_modules/FAST_LIO/PCD/"

# 检查 YAML 文件是否存在
if [ ! -f $YAML_FILE ]; then
  echo "YAML file not found: $YAML_FILE"
  exit 1
fi

# 读取 YAML 文件并提取所有 IP 地址
IP_ADDRESSES=$(yq e '.drone_ip_0, .drone_ip_1, .drone_ip_2' $YAML_FILE)

# 遍历所有 IP 地址并传输文件
for REMOTE_IP in $IP_ADDRESSES; do
  echo "Transferring $LOCAL_FILE to ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}"

  # 使用 scp 传输文件
  scp $LOCAL_FILE ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}

  # 检查 scp 命令是否成功
  if [ $? -eq 0 ]; then
    echo "File transfer to ${REMOTE_IP} successful."
  else
    echo "File transfer to ${REMOTE_IP} failed."
  fi
done

