import os
import subprocess

target_string = "GNF_WS"
ws_path = ""
ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
ros_package_paths = ros_package_path.split(':')
for path in ros_package_paths:
    if target_string in path:
        ws_path = path
        print("Workspace found : ", ws_path)
        break

path_killros   = "sh " + ws_path + "/script/killros.sh"
path_killnode  = "sh " + ws_path + "/script/kill_node.sh"
path_node_init = "roslaunch drone_node run_drone_node_gnf.launch"
path_node_start= ws_path + "/script/start_node.sh"
path_takeoff   = ws_path + "/script/takeoff.sh"
path_land      = ws_path + "/script/land.sh"
path_cmdpub = ws_path + "/script/cmdpub.sh"

res = subprocess.Popen(["sh", path_cmdpub, "0", "4"])
print(res)