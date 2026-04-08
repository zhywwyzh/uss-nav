#!/usr/bin/env python3
import tkinter as tk
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import signal
import sys
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

rospy.init_node('formation_trigger_pub', anonymous=True)
traj_start_pub = rospy.Publisher('/chosen_single_drone_start_trigger', Float64, queue_size=1)
# traj_start_pub = rospy.Publisher('/traj_start_trigger', PoseStamped, queue_size=1)
traj_hover_pub = rospy.Publisher('/traj_hover_trigger', PoseStamped, queue_size=1)
# formation_id_pub = rospy.Publisher('/formation_id_trigger', FormationId, queue_size=1)
current_drone_id = 0
currentflagZ= 1.0
# current_fd = FormationId()

# Define the functions for button events as before ...

def on_start():
    print("Chosen button pressed.")
    # Here you would add your ROS package imports and node initialization
    pos = Float64()
    pos.data=currentflagZ
    colored_print("currentflagZ:{}".format(currentflagZ), "green")
    traj_start_pub.publish(currentflagZ)


def on_stop():
    print("Hover button pressed.")
    # Here you would add your ROS package imports and node initialization
    pos = PoseStamped()
    pos.pose.position.x = 1
    traj_hover_pub.publish(pos)    

# 当前选中的按钮
drone_current_selected_button = None
car_current_selected_button = None

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
    global current_drone_id,currentflagZ
    current_drone_id= id
    currentflagZ= 1.0 + 1000*(current_drone_id+1)
    colored_print("current_drone_id:{}".format(current_drone_id), "green")
    # 更改按钮颜色
    
    change_drone_button_color(drone_buttons[id])

# def change_car_button_color(selected_button):
#     global car_current_selected_button
#     # 将之前的按钮恢复默认颜色
#     if car_current_selected_button is not None:
#         car_current_selected_button.config(bg='white')
#     # 更改当前按钮颜色
#     selected_button.config(bg='gray')
#     # 更新当前选中的按钮
#     car_current_selected_button = selected_button

# def on_car_button_clicked(id):
#     print(f"Button {id} pressed.")
#     current_fd.car_formation_id = id
#     # 更改按钮颜色
#     change_car_button_color(car_buttons[id])


# ... existing button functions ...
    
drone_buttons = []  # 存储所有按钮
car_buttons = []  # 存储所有按钮

# Create the main window
root = tk.Tk()
root.title("Formation GUI")

# Create buttons and assign them to the grid
start_button = tk.Button(root, text="Single_goal_chosen", command=on_start)
start_button.grid(row=0, column=1, columnspan=3, sticky="ew")

# stop_button = tk.Button(root, text="Stop", command=on_stop)
# stop_button.grid(row=0, column=3, columnspan=2, sticky="ew")

# Add a label for drone
drone_label = tk.Label(root, text="Drone")
drone_label.grid(row=1, column=0, columnspan=5)

for i in range(5):
    button = tk.Button(root, text=str(i), command=lambda i=i: on_drone_button_clicked(i),bg="white")
    button.grid(row=2, column=i, sticky="ew")
    drone_buttons.append(button)

# Add a label for car
# car_label = tk.Label(root, text="Car")
# car_label.grid(row=3, column=0, columnspan=5)

# for i in range(5):
#     button = tk.Button(root, text=str(i), command=lambda i=i: on_car_button_clicked(i),bg="white")
#     button.grid(row=4, column=i, sticky="ew")
#     car_buttons.append(button)

def sigint_handler(signum, frame):
    print("Ctrl+C detected, exiting...")
    sys.exit(0)


signal.signal(signal.SIGINT, sigint_handler)

# Start the GUI loop
root.mainloop()