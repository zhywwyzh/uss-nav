#!/usr/bin/env python3
import tkinter as tk
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import signal
import sys
from quadrotor_msgs.msg import SwarmInfo
rospy.init_node('formation_trigger_pub', anonymous=True)
SwamInfo_pub = rospy.Publisher('/Swarm', SwarmInfo, queue_size=1)

def colored_print(text, color):
    colors = {
        'red': '\033[91m',
        'green': '\033[92m',
        'yellow': '\033[93m',
        'blue': '\033[94m',
        'magenta': '\033[95m',
        'cyan': '\033[96m',
        'white': '\033[97m'
    }
    end_color = '\033[0m'
    color_code = colors.get(color.lower(), '')
    if color_code:
        print(f"{color_code}{text}{end_color}")
    else:
        print(text)

# SwamInfo_pub = rospy.Publisher('/traj_start_trigger', PoseStamped, queue_size=1)
# traj_hover_pub = rospy.Publisher('/traj_hover_trigger', PoseStamped, queue_size=1)
# formation_id_pub = rospy.Publisher('/formation_id_trigger', FormationId, queue_size=1)
SwarmId=0
RobotId=[]
LeaderID=0
# current_fd = FormationId()

# Define the functions for button events as before ...

def on_start():
    print("Chosen button pressed.")
    # Here you would add your ROS package imports and node initialization
    swarminfo = SwarmInfo()
    swarminfo.swarm_id=SwarmId
    for i in RobotId:
        swarminfo.robot_ids.append(i)
    swarminfo.leader_id=LeaderID
    colored_print(f"SwarmInfo Id: {SwarmId}, RobotId: {RobotId}, LeaderID: {LeaderID} pressed.", "red")
    SwamInfo_pub.publish(swarminfo)


def on_stop():
    print("Hover button pressed.")
    # Here you would add your ROS package imports and node initialization
    pos = PoseStamped()
    pos.pose.position.x = 1


# 当前选中的按钮
drone_current_selected_button = None
swarmid_current_selected_button = None
leader_current_selected_button=None
def change_drone_cancel_button_color(selected_button):
    selected_button.config(bg='white')
def change_drone_select_button_color(selected_button):
    selected_button.config(bg='gray')

def on_drone_button_clicked(id):
    global RobotId
    colored_print(f"Drone Chosen button {id} pressed.", "green")
    if id in RobotId:
        change_drone_cancel_button_color(drone_buttons[id])
        RobotId.remove(id)
    else:
        change_drone_select_button_color(drone_buttons[id])
        RobotId.append(id)

    
def change_swarm_button_color(selected_button):
    global swarmid_current_selected_button
    # 将之前的按钮恢复默认颜色
    if swarmid_current_selected_button is not None:
        swarmid_current_selected_button.config(bg='white')
    # 更改当前按钮颜色
    selected_button.config(bg='gray')
    # 更新当前选中的按钮
    swarmid_current_selected_button = selected_button

def on_swarmid_button_clicked(id):
    global SwarmId
    colored_print(f"SwarmId Button {id} pressed.", "yellow")
    # 更改按钮颜色
    SwarmId=id
    change_swarm_button_color(swarmid_buttons[id])

def change_leader_button_color(selected_button):
    global leader_current_selected_button
    # 将之前的按钮恢复默认颜色
    if leader_current_selected_button is not None:
        leader_current_selected_button.config(bg='white')
    # 更改当前按钮颜色
    selected_button.config(bg='gray')
    # 更新当前选中的按钮
    leader_current_selected_button = selected_button

def on_leader_button_clicked(id):
    global LeaderID
    colored_print(f"Leader Button {id} pressed.", "blue")
    LeaderID=id
    # 更改按钮颜色
    change_leader_button_color(leader_buttons[id])


# ... existing button functions ...
    
drone_buttons = []  # 存储所有按钮
swarmid_buttons = []  # 存储所有按钮
leader_buttons = []  # 存储所有按钮

# Create the main window
root = tk.Tk()
root.title("Formation GUI")

# Create buttons and assign them to the grid
start_button = tk.Button(root, text="SwarmInfo", command=on_start)
start_button.grid(row=0, column=1, columnspan=3, sticky="ew")

# stop_button = tk.Button(root, text="Stop", command=on_stop)
# stop_button.grid(row=0, column=3, columnspan=2, sticky="ew")

# Add a label for drone
drone_label = tk.Label(root, text="RobotsIds")
drone_label.grid(row=2, column=0, columnspan=5)

for i in range(10):
    button = tk.Button(root, text=str(i), command=lambda i=i: on_drone_button_clicked(i),bg="white")
    button.grid(row=3, column=i, sticky="ew")
    drone_buttons.append(button)

# Add a label for swarmId
swarm_label = tk.Label(root, text="SwarmId")
swarm_label.grid(row=4, column=0, columnspan=5)

for i in range(5):
    button = tk.Button(root, text=str(i), command=lambda i=i: on_swarmid_button_clicked(i),bg="white")
    button.grid(row=5, column=i, sticky="ew")
    swarmid_buttons.append(button)

# Add a label for car
type_label = tk.Label(root, text="Leader")
type_label.grid(row=6, column=0, columnspan=6)

for i in range(10):
    button = tk.Button(root, text=str(i), command=lambda i=i: on_leader_button_clicked(i),bg="white")
    button.grid(row=7, column=i, sticky="ew")
    leader_buttons.append(button)

def sigint_handler(signum, frame):
    print("Ctrl+C detected, exiting...")
    sys.exit(0)


signal.signal(signal.SIGINT, sigint_handler)

# Start the GUI loop
root.mainloop()