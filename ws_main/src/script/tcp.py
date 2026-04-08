import asyncio
import websockets
import threading
import json
import os
import subprocess
from time import sleep

target_string = "GNF_WS"
ws_path = ""
ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
ros_package_paths = ros_package_path.split(':')
for path in ros_package_paths:
    if target_string in path:
        ws_path = path
        print("Workspace found : ", ws_path)
        break

path_killros = ws_path + "/script/killros.sh"
path_killnode = ws_path + "/script/kill_node.sh"
path_node_init = ws_path + "/script/node_init.sh"
path_node_start = ws_path + "/script/start_node.sh"
path_planner_start = ws_path + "/script/run_planner.sh"
path_takeoff = ws_path + "/script/takeoff.sh"
path_land = ws_path + "/script/land.sh"
path_cmdpub = ws_path + "/script/cmdpub.sh"

drone_id = env_var_value = os.environ.get('DRONE_ID')
if drone_id:
    drone_id = str(drone_id)
    print("Drone ID found: ", drone_id)
else:
    drone_id = "0"
    print("Drone ID not found, set to 0")
explore = None
patrol = None


async def handler(websocket, path):
    print("Client connected")  # 客户端连接后打印消息
    receive_task = asyncio.create_task(receive_data(websocket))
    await receive_task


async def receive_data(websocket):
    global explore
    global patrol
    while True:
        sleep(0.05)
        try:
            message = await websocket.recv()
            data = json.loads(message)
            print("Received message:", data)
            # TODO：加入具体功能
            if data['type'] == 'sh':
                # sh指令，调用脚本
                cmd = data['data']
                if cmd == 1:
                    # node
                    res = subprocess.Popen(["sh", path_node_init])
                    print("所有无人机开机")
                if cmd == 2:
                    # killros
                    res = subprocess.Popen(["sh", path_killros]).wait()
                    print("所有无人机关机")
            elif data['type'] == 'command':
                cmd = data['data']
                if cmd == 1:
                    res = subprocess.Popen(["sh", path_node_start]).wait()
                    res = subprocess.Popen(["sh", path_planner_start])
                    print("加载系统")
                if cmd == 2:
                    res = subprocess.Popen(["sh", path_killnode]).wait()
                    print("停止系统")
                if cmd == 3:
                    res = subprocess.Popen(["sh", path_takeoff]).wait()
                    print("起飞")
                if cmd == 4:
                    res = subprocess.Popen(["sh", path_land]).wait()
                    print("降落")
                if cmd == 5:
                    res = subprocess.Popen(["sh", path_cmdpub, "0", "4"]).wait()
                    print("回家")
                if cmd == 9:
                    res = subprocess.Popen(["sh", path_cmdpub, "0", "2"]).wait()
                    print("探索")
            elif data['type'] == 'update_explore':
                print("更新巡逻框")
                yaml_data = data['data']
                explore = yaml_data.encode('utf-8')
                # 写入文件
                # with open("explore_para.yaml", 'w') as yaml_file:
                #     yaml_file.write(yaml_data)
                print(explore)
            elif data['type'] == 'update_patrol':
                print("更新探索点")
                txt_data = data['data']
                patrol = txt_data.encode('utf-8')
                print(patrol)
        except websockets.ConnectionClosed:
            print("Connection closed")
            break


async def main():
    server = await websockets.serve(handler, "192.168.100.124", 8080)
    print("WebSocket server started at ws://192.168.100.124:8080")
    await server.wait_closed()


def start_asyncio_server(loop):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
    loop.run_forever()


if __name__ == "__main__":
    # 创建一个事件循环并在后台线程中启动它
    loop = asyncio.new_event_loop()
    asyncio_thread = threading.Thread(target=start_asyncio_server, args=(loop,), daemon=True)
    asyncio_thread.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Server stopped")
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()
        print("Event loop closed")
