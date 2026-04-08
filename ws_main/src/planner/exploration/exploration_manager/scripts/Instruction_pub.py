#!/usr/bin/env python3
import tkinter as tk
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16MultiArray
import signal
import sys
from quadrotor_msgs.msg import Instruction

rospy.init_node('formation_trigger_pub', anonymous=True)
Instruction_pub = rospy.Publisher('/Instruct', Instruction, queue_size=1)


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


# Instruction_pub = rospy.Publisher('/traj_start_trigger', PoseStamped, queue_size=1)
# traj_hover_pub = rospy.Publisher('/traj_hover_trigger', PoseStamped, queue_size=1)
# formation_id_pub = rospy.Publisher('/formation_id_trigger', FormationId, queue_size=1)

RobotId = 0
InstructionType = 0


# current_fd = FormationId()

# Define the functions for button events as before ...

def on_start():
    print("Instruction button pressed.")
    # Here you would add your ROS package imports and node initialization
    Ins = Instruction()
    Ins.robot_id = RobotId
    Ins.instruction_type = InstructionType + 1
    if InstructionType + 1 == 9:
        Ins.tar_drone_ids.append(0)
        Ins.src_drone_ids.append(1)
    elif InstructionType + 1 == 10:
        Ins.tar_drone_ids.append(1)
    colored_print(f"Instruction RobotId: {RobotId}, Instruction: {InstructionType + 1}.", "red")
    Instruction_pub.publish(Ins)


def on_stop():
    print("Hover button pressed.")
    # Here you would add your ROS package imports and node initialization
    pos = PoseStamped()
    pos.pose.position.x = 1


# 当前选中的按钮
drone_current_selected_button = None
Instruc_Id_current_selected_button = None


def change_drone_button_color(selected_button):
    global drone_current_selected_button
    # 将之前的按钮恢复默认颜色
    if drone_current_selected_button is not None:
        drone_current_selected_button.config(bg='white')
    # 更改当前按钮颜色
    selected_button.config(bg='gray')
    # 更新当前选中的按钮
    drone_current_selected_button = selected_button


def on_drone_button_clicked(id):
    global RobotId
    colored_print(f"Drone Chosen button {id} pressed.", "green")
    RobotId = id

    # 更改按钮颜色

    change_drone_button_color(drone_buttons[id])


def change_instruc_button_color(selected_button):
    global Instruc_Id_current_selected_button
    # 将之前的按钮恢复默认颜色
    if Instruc_Id_current_selected_button is not None:
        Instruc_Id_current_selected_button.config(bg='white')
    # 更改当前按钮颜色
    selected_button.config(bg='gray')
    # 更新当前选中的按钮
    Instruc_Id_current_selected_button = selected_button


def on_instruction_button_clicked(id):
    global InstructionType
    colored_print(f"Instruction Button {id + 1} pressed.", "yellow")
    # 更改按钮颜色
    InstructionType = id
    change_instruc_button_color(Instru_id_buttons[id])


# ... existing button functions ...

drone_buttons = []  # 存储所有按钮
Instru_id_buttons = []  # 存储所有按钮
rolytype_buttons = []  # 存储所有按钮

# Create the main window
root = tk.Tk()
root.title("Formation GUI")

# Create buttons and assign them to the grid
start_button = tk.Button(root, text="Instruction", command=on_start)
start_button.grid(row=0, column=1, columnspan=3, sticky="ew")

# stop_button = tk.Button(root, text="Stop", command=on_stop)
# stop_button.grid(row=0, column=3, columnspan=2, sticky="ew")

# Add a label for drone
drone_label = tk.Label(root, text="RobotsIds")
drone_label.grid(row=2, column=0, columnspan=5)

for i in range(6):
    button = tk.Button(root, text=str(i), command=lambda i=i: on_drone_button_clicked(i), bg="white")
    button.grid(row=3, column=i, sticky="ew")
    drone_buttons.append(button)

# Add a label for swarmId
instruc_label = tk.Label(root, text="Instruction")
instruc_label.grid(row=4, column=0, columnspan=6)

for i in range(10):
    button = tk.Button(root, text=str(i + 1), command=lambda i=i: on_instruction_button_clicked(i), bg="white")
    button.grid(row=5, column=i, sticky="ew")
    Instru_id_buttons.append(button)


# map_merge_target_label = tk.Label(root, text="MapMergeTarget")
# map_merge_target_label.grid(row=6, column=0, columnspan=6)
# for i in range(6):
#     button = tk.Button(root, text=str(i), command=lambda i=i: on_drone_button_clicked(i),bg="white")
#     button.grid(row=7, column=i, sticky="ew")
#     drone_buttons.append(button)

# map_merge_target_label = tk.Label(root, text="MapMergeSource")
# map_merge_target_label.grid(row=8, column=0, columnspan=6)
# for i in range(6):
#     button = tk.Button(root, text=str(i), command=lambda i=i: on_drone_button_clicked(i),bg="white")
#     button.grid(row=9, column=i, sticky="ew")
#     drone_buttons.append(button)


def sigint_handler(signum, frame):
    print("Ctrl+C detected, exiting...")
    sys.exit(0)


signal.signal(signal.SIGINT, sigint_handler)

# Start the GUI loop
root.mainloop()
