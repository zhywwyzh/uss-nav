#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
predict_realtime_cam_real_tcp.py
================================
YOLO 实机版实时推理服务（纯 Python、非 ROS 外部进程，机载运行）。

是 ros_ws/uss-nav/yoloe/predict_realtime_cam_real.py 的 TCP 解耦版：
- 本脚本作为 TCP server，接收广播器（C++ ROS 节点）通过 TCP 发来的
  rgb / depth / odom 三路帧数据；
- 模型使用 **YOLO 类**（非 YOLOE），支持 TensorRT .engine 推理与 engine
  导出（export_mode）；
- 里程计走独立缓冲 + 时间戳插值（位置线性 + 四元数 slerp），以 rgb 时间戳
  为基准取同步 odom；
- CLIP 文本编码按 label 缓存，只对未见过的词编码；
- 可选可视化：YOLO plot + rgb + 深度伪彩色三图拼接后 cv2.imshow。
- 推理结果（原始 rgb/depth 压缩字节 + 各目标 mask PNG / 512 维 word_vector）
  通过同一 TCP 连接以 JSON 形式回发给广播器，由广播器重建 EncodeMask 消息。

本脚本不依赖 rospy / cv_bridge，仅依赖 opencv / numpy / torch /
ultralytics(YOLO) / mobileclip / base64 / json / struct / socket / threading。

帧协议（与广播器 C++ 端共享，全部大端序 / network order，与
yoloe_remote_service.py 完全一致）：
    头部固定 4+1+1+2+8+8+4 = 28 字节：
        magic     : 4 字节  b"TLBC"
        version   : 1 字节  1
        direction : 1 字节  0=上行(广播器->本服务), 1=下行(本服务->广播器)
        topic_id  : 2 字节  uint16 大端: 1=rgb, 2=depth, 3=odom, 100=EncodeMask 结果
        seq       : 8 字节  uint64 大端, 每话题独立递增
        stamp     : 8 字节  double 大端, 单位秒
        payload_len : 4 字节 uint32 大端
        payload   : payload_len 字节
"""

import argparse
import base64
import json
import socket
import struct
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import torch
import mobileclip
from ultralytics import YOLO

# ------------------------------ 帧协议常量 ------------------------------ #
MAGIC = b"TLBC"               # 帧魔数，用于识别帧起始
VERSION = 1                   # 协议版本号
DIR_UP = 0                    # 上行：广播器 -> 本服务
DIR_DOWN = 1                  # 下行：本服务 -> 广播器
TOPIC_RGB = 1                 # 上行：rgb 压缩图（jpeg）
TOPIC_DEPTH = 2               # 上行：depth 压缩图（png）
TOPIC_ODOM = 3                # 上行：odom JSON
TOPIC_RESULT = 100            # 下行：EncodeMask 结果 JSON
HEADER_STRUCT = ">4sBBHQdI"   # 帧头打包格式：magic(4s) version(B) direction(B) topic(H) seq(Q) stamp(d) payload_len(I)
HEADER_SIZE = struct.calcsize(HEADER_STRUCT)  # 28 字节

# compressedDepth 传输的 12 字节配置头（小端 <iff，参考 fake_realtime_cam_real.py）
COMPRESSED_DEPTH_HEADER = struct.pack("<iff", 0, 0.0, 0.0)


class FrameParser:
    """TCP 字节流帧解析器：处理粘包 / 半包（实现与 yoloe_remote_service.py 一致）。

    内部维护一个 bytearray 缓冲，调用方将 recv 到的原始字节通过 feed()
    追加进来，再循环调用 next_frame() 逐个取出完整帧。
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        """追加收到的原始字节到内部缓冲。"""
        self._buf.extend(data)

    def next_frame(self):
        """尝试从缓冲中解析出一个完整帧。

        返回 dict: {"direction", "topic_id", "seq", "stamp", "payload(bytes)"}
        数据不足（半包）时返回 None 并保留缓冲，等待下次 feed 后再次调用。
        若缓冲头部魔数不匹配，则向前搜索下一个 b"TLBC" 丢弃坏数据（鲁棒性处理）。
        """
        while True:
            # 缓冲不足一个帧头，属于半包，等待更多数据
            if len(self._buf) < HEADER_SIZE:
                return None

            # 帧头魔数校验：不匹配说明字节流错位或收到脏数据
            if self._buf[:4] != MAGIC:
                idx = self._buf.find(MAGIC, 1)
                if idx == -1:
                    # 缓冲中没有完整魔数：丢弃除末尾 3 字节外的数据，
                    # 保留末尾防止 b"TLBC" 跨包分裂
                    drop = len(self._buf) - (len(MAGIC) - 1)
                    if drop > 0:
                        del self._buf[:drop]
                    return None
                print(f"[FrameParser] 帧头魔数不匹配，丢弃 {idx} 字节坏数据")
                del self._buf[:idx]
                continue

            # 解析帧头（全部大端序）
            _magic, _version, direction, topic_id, seq, stamp, payload_len = \
                struct.unpack_from(HEADER_STRUCT, self._buf, 0)

            total = HEADER_SIZE + payload_len
            # 载荷尚未收齐，属于半包，保留缓冲等待后续数据
            if len(self._buf) < total:
                return None

            # 取出完整一帧并消费对应缓冲
            payload = bytes(self._buf[HEADER_SIZE:total])
            del self._buf[:total]
            return {
                "direction": direction,
                "topic_id": topic_id,
                "seq": seq,
                "stamp": stamp,
                "payload": payload,
            }


class OdometryBuffer:
    """里程计缓冲与插值（纯 Python 版，照搬 ROS 实机版的插值算法）。

    内部按时间顺序保存 (stamp_sec, [x,y,z,qx,qy,qz,qw])，支持按目标时间戳
    取插值结果：位置线性插值 + 四元数球面插值（slerp）。
    """

    def __init__(self, max_len=1000, sync_slop=0.005):
        """
        :param max_len:   缓冲队列最大长度（deque 自动覆盖最旧数据）
        :param sync_slop: 最大允许的时间同步误差（秒），超出则视为无法插值
        """
        self.lock = threading.Lock()
        self.buffer = deque(maxlen=max_len)
        self.sync_slop = sync_slop

    def add_odom(self, stamp, pose):
        """将一帧 (stamp, pose) 加入缓冲。"""
        with self.lock:
            self.buffer.append((stamp, pose))

    def get_interpolated(self, target_time):
        """根据目标时间戳计算插值后的里程计 [x,y,z,qx,qy,qz,qw]。

        :param target_time: 目标时间戳（秒，float）
        :return: 插值后的 7 维 pose 列表；缓冲不足或超出容差返回 None
        """
        with self.lock:
            if len(self.buffer) < 2:
                return None

            newest_time = self.buffer[-1][0]
            oldest_time = self.buffer[0][0]

            # 目标时间超出缓冲范围且超过容差，无法插值
            if target_time > newest_time + self.sync_slop:
                return None
            if target_time < oldest_time - self.sync_slop:
                return None

            # 寻找最后一个 stamp <= 目标时间的帧（缓冲数据按时间有序）
            idx = -1
            for i in range(len(self.buffer) - 1, -1, -1):
                if self.buffer[i][0] <= target_time:
                    idx = i
                    break

            # Case A: 目标时间比最早的还早，但在容差范围内 -> 返回最早那帧
            if idx == -1:
                return list(self.buffer[0][1])

            # Case B: 目标时间比最新的还晚，但在容差范围内 -> 返回最新那帧
            if idx == len(self.buffer) - 1:
                return list(self.buffer[-1][1])

            # Case C: 正常插值范围（取前后两帧，释放锁后再计算减少阻塞）
            t0, pose0 = self.buffer[idx]
            t1, pose1 = self.buffer[idx + 1]

        return self._interpolate(t0, pose0, t1, pose1, target_time)

    def _interpolate(self, t0, pose0, t1, pose1, target_time):
        """位置线性插值 + 四元数 slerp（照搬 ROS 实机版算法）。"""
        # 防止除以零
        if abs(t1 - t0) < 1e-9:
            return list(pose0)

        alpha = (target_time - t0) / (t1 - t0)

        # --- 位置插值 (Linear) ---
        res_pose = [0.0] * 7
        for i in range(3):
            res_pose[i] = pose0[i] + alpha * (pose1[i] - pose0[i])

        # --- 姿态插值 (Slerp) ---
        q0 = np.array(pose0[3:7], dtype=np.float64)
        q1 = np.array(pose1[3:7], dtype=np.float64)
        q_res = self._slerp(q0, q1, alpha)
        res_pose[3:7] = q_res.tolist()
        return res_pose

    @staticmethod
    def _slerp(q0, q1, t):
        """四元数球面线性插值（照搬 ROS 实机版算法）。"""
        # 归一化
        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)

        dot = np.dot(q0, q1)

        # 如果点积为负，反转 q1 以走最短路径
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        # 如果太接近，直接线性插值避免数值不稳定
        if dot > 0.9995:
            result = q0 + t * (q1 - q0)
            return result / np.linalg.norm(result)

        theta_0 = np.arccos(dot)        # theta_0 = 两个输入向量的夹角
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q0) + (s1 * q1)


class YoloRealTcpService:
    """YOLO 实机版 TCP 推理服务。

    作为 TCP server 接收广播器上行帧（rgb/depth/odom）：
    - odom 高频入 OdometryBuffer，rgb 触发帧时以 rgb 时间戳插值取同步 odom；
    - rgb/depth 时间同步由 time_slop 控制；
    - 结果通过下行帧（topic_id=100, JSON）回发给广播器。
    """

    def __init__(self, host="127.0.0.1", port=9010,
                 prompt_model_path="./yoloe-v8m-seg2.pt",
                 prompt_file="./prompt/prompt.txt",
                 clip_model_type="mobileclip_b",
                 clip_model_path="./mobileclip_blt.pt",
                 use_prompt=True, use_clip=True,
                 export_mode=True, use_tensorrt=None,
                 resize_width=640, imgsz_h=480, imgsz_w=640,
                 conf=0.4, time_slop=0.005, odom_sync_slop=0.005,
                 visualize=False, visualize_scale=0.5,
                 device="cuda:0", debug=False):
        # ---- 网络与推理参数 ----
        self.host = host
        self.port = port
        self.conf = conf                    # model.predict 的置信度阈值
        self.resize_width = resize_width    # 推理前将 RGB 宽度缩放到该值
        self.imgsz_h = imgsz_h              # 推理 / 导出固定输入高（默认 480）
        self.imgsz_w = imgsz_w              # 推理 / 导出固定输入宽（默认 640）
        self.time_slop = time_slop          # RGB-D 时间同步容差（秒）
        self.odom_sync_slop = odom_sync_slop  # odom 插值最大允许时间差（秒）
        self.device = device
        self.visualize = visualize          # 是否显示可视化窗口
        self.visualize_scale = visualize_scale  # 可视化缩放比例
        self.debug = debug

        # ---- 模型路径与开关（默认值与 ROS 实机版一致） ----
        self.prompt_model_path = prompt_model_path
        self.prompt_file = prompt_file
        self.clip_model_type = clip_model_type
        self.clip_model_path = clip_model_path
        self.use_prompt = use_prompt
        self.use_clip = use_clip
        self.export_mode = export_mode
        # use_tensorrt: None 表示自动判断（模型路径含 .engine 则启用），
        # True / False 为显式强制
        self.use_tensorrt = use_tensorrt

        # ---- 里程计缓冲与 CLIP 缓存 ----
        self.odom_buffer = OdometryBuffer(max_len=1000, sync_slop=self.odom_sync_slop)
        self.clip_cache = {}                # {label: 512 维归一化特征}

        # ---- 最近一帧的 rgb / depth 缓存 ----
        self._latest_rgb = None    # {"stamp", "bgr", "payload"}
        self._latest_depth = None  # {"stamp", "array", "payload"}

        # ---- 并发控制 ----
        self._infer_lock = threading.Lock()   # 保证同一时刻只有一个推理线程
        self._send_lock = threading.Lock()    # 下行发送加锁，防止多线程交错写 socket
        self._result_seq = 0                  # 下行 EncodeMask 帧的 seq（独立自增）
        self._conn = None                     # 当前连接句柄（单连接模式）

        # 模型加载：失败则打印清晰错误并退出（无模型时服务无意义）
        try:
            self._load_models()
        except Exception as e:
            print(f"[YoloRealTcpService] 模型加载失败: {e}")
            sys.exit(1)

    # ------------------------------------------------------------------ #
    # 模型加载（逻辑照搬 ROS 实机版 predict_realtime_cam_real.py 的 __init__）
    # ------------------------------------------------------------------ #
    def _load_models(self):
        if not torch.cuda.is_available():
            print("[YoloRealTcpService] CUDA 不可用，无法运行 YOLO 推理")
            sys.exit(1)

        print("[YoloRealTcpService] Model INIT ...")
        if self.use_prompt:
            with open(self.prompt_file, "r") as f:
                self.names = [x.strip() for x in f.readlines()]

            # TensorRT 判断：显式参数优先，否则按模型路径是否含 .engine 自动判断
            use_tensorrt = self.use_tensorrt
            if use_tensorrt is None:
                use_tensorrt = '.engine' in self.prompt_model_path

            if use_tensorrt:
                print("[YoloRealTcpService] ********** USE TENSOR-RT **********")
                self.model = YOLO(self.prompt_model_path)
            else:
                self.model = YOLO(self.prompt_model_path).cuda()
                # YOLO 实机版使用 text_pe 直接设置类别，不走 YOLOE 的 set_vocab
                self.model.set_classes(self.names, self.model.get_text_pe(self.names))
        else:
            self.model = YOLO(self.prompt_model_path).cuda()

        # 导出 TensorRT engine（照搬 ROS 实机版：固定 imgsz=(h, w)=(480, 640)，
        # 强制关闭动态形状，FP16 + ONNX simplify）
        if self.export_mode:
            export_model = self.model.export(
                format="engine",
                imgsz=(self.imgsz_h, self.imgsz_w),   # 显式指定 高, 宽
                dynamic=False,        # 强制关闭动态形状
                half=True,            # 开启 FP16 加速
                simplify=True         # 简化 ONNX 结构
            )
            print(f"[YoloRealTcpService] Exported model path: {export_model}")

        # MobileCLIP 文本编码器（生成 512 维 word_vector）
        if self.use_clip:
            self.clip_model, _, self.clip_preprocess = \
                mobileclip.create_model_and_transforms(self.clip_model_type,
                                                       pretrained=self.clip_model_path)
            self.clip_model.to("cuda")
            self.clip_model.eval()
            self.clip_tokenizer = mobileclip.get_tokenizer(self.clip_model_type)

        # 模型预热（照搬 ROS 实机版的 model_heat）
        self._model_heat()

        print("[YoloRealTcpService] 模型初始化完成")

    def _model_heat(self):
        """模型预热：用示例图跑推理，CLIP 同样预热（与 ROS 实机版一致）。"""
        img_input = cv2.imread('./ultralytics/assets/bus.jpg')
        if img_input is None:
            print("[YoloRealTcpService] 预热图 ./ultralytics/assets/bus.jpg 不存在，跳过模型预热")
            return
        img_input = cv2.resize(img_input, (self.imgsz_w, self.imgsz_h))
        with torch.no_grad():
            for _ in range(2):
                for _ in range(10):
                    self.model.predict(img_input, conf=0.3, imgsz=(self.imgsz_h, self.imgsz_w),
                                       device=self.device, save=False, verbose=False)
        if self.use_clip:
            text = self.clip_tokenizer(["a diagram", "a dog"]).to(self.device)
            with torch.no_grad():
                for _ in range(20):
                    self.clip_model.encode_text(text)

    # ------------------------------------------------------------------ #
    # TCP server 主流程
    # ------------------------------------------------------------------ #
    def start(self):
        """创建 TCP server 并循环接受广播器连接（单连接模式）。"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[YoloRealTcpService] 开始监听 {self.host}:{self.port} "
              f"(time_slop={self.time_slop}s, odom_sync_slop={self.odom_sync_slop}s)")
        try:
            while True:
                conn, addr = server.accept()
                print(f"[YoloRealTcpService] 广播器已连接: {addr}")
                # 单连接模式：若上一连接仍有残留，先关闭再处理新连接
                if self._conn is not None:
                    try:
                        self._conn.close()
                    except OSError:
                        pass
                self._conn = conn
                self._handle_connection(conn)
                print(f"[YoloRealTcpService] 广播器连接断开: {addr}")
        except KeyboardInterrupt:
            # 仅处理 Ctrl-C 优雅退出，不吞掉其他异常
            print("[YoloRealTcpService] 收到 Ctrl-C，服务退出")
        finally:
            server.close()

    def _handle_connection(self, conn):
        """单个连接的处理循环：收流 -> 解析帧 -> 按 topic 分发。"""
        parser = FrameParser()
        while True:
            try:
                data = conn.recv(65536)
            except OSError as e:
                # socket 断开 / 出错是必须处理的异常边界
                print(f"[YoloRealTcpService] recv 出错，连接终止: {e}")
                break
            if not data:
                # 对端关闭连接
                print("[YoloRealTcpService] 对端关闭连接")
                break

            parser.feed(data)
            # 缓冲中可能有多帧，循环取帧直到半包（返回 None）
            while True:
                frame = parser.next_frame()
                if frame is None:
                    break
                self._dispatch(frame, conn)

    def _dispatch(self, frame, conn):
        """按 topic_id 分发上行帧数据。"""
        topic_id = frame["topic_id"]

        if topic_id == TOPIC_RGB:
            # rgb：jpeg 压缩字节 -> cv2.imdecode 得到 BGR 图
            bgr = cv2.imdecode(np.frombuffer(frame["payload"], np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                print("[YoloRealTcpService] rgb jpeg 解码失败，丢弃该帧")
                return
            self._latest_rgb = {
                "stamp": frame["stamp"],
                "bgr": bgr,
                "payload": frame["payload"],   # 保留原始压缩字节用于回发
            }
            # rgb 是触发帧：收到后尝试一次推理
            self._try_infer(conn)

        elif topic_id == TOPIC_DEPTH:
            # depth：png 压缩字节 -> cv2.imdecode(IMREAD_UNCHANGED)
            depth_payload = frame["payload"]
            # 鲁棒性处理：实机 Realsense 的 compressedDepth 数据前有 12 字节
            # 配置头（struct.pack("<iff", 0, 0.0, 0.0)），需跳过后再解码
            if len(depth_payload) > 12 and depth_payload[:12] == COMPRESSED_DEPTH_HEADER:
                if self.debug:
                    print("[YoloRealTcpService][debug] 检测到 compressedDepth 12 字节头，跳过")
                depth_payload = depth_payload[12:]
            depth = cv2.imdecode(np.frombuffer(depth_payload, np.uint8), cv2.IMREAD_UNCHANGED)
            if depth is None:
                print("[YoloRealTcpService] depth png 解码失败，丢弃该帧")
                return
            self._latest_depth = {
                "stamp": frame["stamp"],
                "array": depth,
                "payload": frame["payload"],   # 保留原始压缩字节（含头部）用于回发
            }

        elif topic_id == TOPIC_ODOM:
            # odom：UTF-8 JSON {"x":..,"y":..,"z":..,"qx":..,"qy":..,"qz":..,"qw":..}
            try:
                odom = json.loads(frame["payload"].decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                print(f"[YoloRealTcpService] odom JSON 解析失败，丢弃该帧: {e}")
                return
            pose = [
                float(odom.get("x", 0.0)), float(odom.get("y", 0.0)), float(odom.get("z", 0.0)),
                float(odom.get("qx", 0.0)), float(odom.get("qy", 0.0)), float(odom.get("qz", 0.0)),
                float(odom.get("qw", 1.0)),
            ]
            # 高频 odom 全部入缓冲，触发帧时按时间戳插值取同步结果
            self.odom_buffer.add_odom(frame["stamp"], pose)

        else:
            print(f"[YoloRealTcpService] 未知 topic_id={topic_id}，丢弃该帧")

    # ------------------------------------------------------------------ #
    # 推理触发与执行
    # ------------------------------------------------------------------ #
    def _try_infer(self, conn):
        """rgb 触发帧处理：RGB-D 同步 + odom 插值后，异步启动推理线程。"""
        if self._latest_rgb is None or self._latest_depth is None:
            # rgb / depth 尚未齐备，等待后续帧
            return

        rgb_stamp = self._latest_rgb["stamp"]
        # RGB-D 时间同步校验：depth 与 rgb 的 stamp 差需在 time_slop 内
        if abs(self._latest_depth["stamp"] - rgb_stamp) > self.time_slop:
            if self.debug:
                print(f"[YoloRealTcpService][debug] 等待 RGB-D 同步: "
                      f"rgb={rgb_stamp:.3f} depth={self._latest_depth['stamp']:.3f}")
            return

        # 以 rgb 时间戳为基准，从缓冲中插值取同步 odom；
        # 缓冲不足或超出容差则跳过该帧（保证数据质量）
        odom = self.odom_buffer.get_interpolated(rgb_stamp)
        if odom is None:
            if self.debug:
                print(f"[YoloRealTcpService][debug] Odom 插值失败 for time {rgb_stamp:.3f}，"
                      f"缓冲不足或 laggy，跳过该帧")
            return

        # 同一时刻只允许一个推理线程：上一帧推理未结束时跳过当前触发帧
        if not self._infer_lock.acquire(blocking=False):
            if self.debug:
                print("[YoloRealTcpService][debug] 上一帧推理仍在进行，跳过当前触发帧")
            return

        rgb = self._latest_rgb
        depth = self._latest_depth
        # 异步推理：避免阻塞收帧循环（daemon 线程）
        threading.Thread(target=self._run_infer, args=(conn, rgb, depth, odom), daemon=True).start()

    def _run_infer(self, conn, rgb, depth, odom):
        """推理线程入口：执行推理并在有结果时回发，最后释放推理锁。"""
        try:
            result = self._infer(rgb["bgr"], depth["array"], odom, rgb["stamp"],
                                 rgb["payload"], depth["payload"])
            if result is not None:
                self._send_result(conn, result)
        except Exception as e:
            # 推理线程内的兜底异常打印，避免线程静默死亡
            print(f"[YoloRealTcpService] 推理线程异常: {e}")
        finally:
            self._infer_lock.release()

    def _infer(self, rgb_bgr, depth, odom, stamp, rgb_bytes, depth_bytes):
        """核心推理（逻辑照搬 ROS 实机版 processing_loop 的推理部分）。

        参数:
            rgb_bgr: BGR 图像（由 jpeg 解码得到）
            depth  : 深度图数组（用于可视化；uint16 时先做距离截断再归一化）
            odom   : [x, y, z, qx, qy, qz, qw]（已按 rgb 时间戳插值）
            stamp  : rgb 帧的时间戳（作为结果基准时间戳）
            rgb_bytes / depth_bytes: 收到的原始压缩字节，base64 后回发给广播器
        返回:
            结果 dict；若模型无检测结果返回 None（不回发）
        """
        # 1. BGR -> RGB（与原实现一致），并缩放至指定宽度保持宽高比
        cv_rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = cv_rgb.shape
        if w != self.resize_width:
            cv_rgb_resized = cv2.resize(cv_rgb, (self.resize_width, int(h * self.resize_width / w)))
        else:
            cv_rgb_resized = cv_rgb

        # 2. YOLO 推理（固定 imgsz，与导出的 engine 一致）
        t1 = time.perf_counter()
        with torch.no_grad():
            results = self.model.predict(
                cv_rgb_resized, conf=self.conf, imgsz=(self.imgsz_h, self.imgsz_w),
                device=self.device, verbose=False, save=False
            )
        if self.debug:
            print(f"[YoloRealTcpService][debug] YOLO time: {((time.perf_counter() - t1) * 1e3):.2f}ms")
        if not results or len(results) == 0:
            return None
        result = results[0]

        # 3. 可视化（可选）：YOLO plot + rgb + 深度伪彩色三图拼接
        if self.visualize:
            try:
                yolo_plot_img = result.plot(boxes=True, masks=True, conf=True, labels=True)
                vis_yolo = cv2.cvtColor(yolo_plot_img, cv2.COLOR_RGB2BGR)
                vis_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)
                vis_depth_temp = depth.copy()
                vis_depth_temp[vis_depth_temp > 10000] = 10000   # 距离截断
                vis_depth_norm = cv2.normalize(vis_depth_temp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                vis_depth = cv2.applyColorMap(vis_depth_norm, cv2.COLORMAP_JET)

                target_h, target_w = vis_yolo.shape[:2]
                if vis_rgb.shape[:2] != (target_h, target_w):
                    vis_rgb = cv2.resize(vis_rgb, (target_w, target_h))
                if vis_depth.shape[:2] != (target_h, target_w):
                    vis_depth = cv2.resize(vis_depth, (target_w, target_h))

                combined_img = np.hstack((vis_yolo, vis_rgb, vis_depth))
                if self.visualize_scale != 1.0:
                    new_w = int(combined_img.shape[1] * self.visualize_scale)
                    new_h = int(combined_img.shape[0] * self.visualize_scale)
                    combined_img = cv2.resize(combined_img, (new_w, new_h))
                cv2.imshow("YOLO Real-Time Detection", combined_img)
                cv2.waitKey(1)
            except Exception as e:
                print(f"[YoloRealTcpService] 可视化异常: {e}")

        # 4. 提取标签 / 置信度
        objects = []
        labels = []
        if hasattr(result, 'boxes') and result.boxes is not None:
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            class_names = [result.names[cls_id] for cls_id in class_ids] if \
                hasattr(result, 'names') else [f"class_{cls_id}" for cls_id in class_ids]
            labels = list(class_names)
            for cls_name, cls_conf in zip(class_names, confidences):
                objects.append({
                    "label": str(cls_name),
                    "conf": float(cls_conf),
                    "mask_b64": None,
                    "word_vector": None,
                })

        # 模型无任何检测结果：返回 None，不回发
        if len(labels) == 0:
            return None

        # 5. 掩码提取：masks.data -> 二值图 -> PNG 压缩
        #    （与原实现一致：binary_mask=(m*255).astype(uint8)，PNG 压缩级别 0）
        if hasattr(result, 'masks') and result.masks is not None:
            mask_arrays = result.masks.data.cpu().numpy()
            for i in range(len(mask_arrays)):
                single_mask = mask_arrays[i]
                binary_mask = (single_mask * 255).astype(np.uint8)
                try:
                    ok, buf = cv2.imencode(".png", binary_mask,
                                           [cv2.IMWRITE_PNG_COMPRESSION, 0])
                    if ok:
                        objects[i]["mask_b64"] = base64.b64encode(buf.tobytes()).decode("utf-8")
                    else:
                        print(f"[YoloRealTcpService] 掩码 {i} PNG 编码失败")
                except Exception as e:
                    # 个别掩码转换失败不应中断整帧处理
                    print(f"[YoloRealTcpService] 掩码 {i} 编码异常: {e}")

        # 6. MobileCLIP 文本编码（带 label 缓存，只对未见过的词编码）
        if self.use_clip:
            try:
                unknown_words = list(set([label for label in labels if label not in self.clip_cache]))
                if len(unknown_words) > 0:
                    text_input = self.clip_tokenizer(unknown_words).to(self.device)
                    with torch.no_grad():
                        new_features = self.clip_model.encode_text(text_input)
                        new_features /= new_features.norm(dim=-1, keepdim=True)
                        new_features = new_features.cpu().numpy()
                    for idx, word in enumerate(unknown_words):
                        self.clip_cache[word] = new_features[idx]
                for i, label in enumerate(labels):
                    if label in self.clip_cache:
                        objects[i]["word_vector"] = self.clip_cache[label].astype(np.float64).tolist()
            except Exception as e:
                print(f"[YoloRealTcpService] CLIP 编码异常: {e}")

        # 7. 组装回发 JSON（rgb/depth 使用收到的原始压缩字节，不重编码）
        result_dict = {
            "stamp": stamp,
            "odom": [float(v) for v in odom],
            "rgb_b64": base64.b64encode(rgb_bytes).decode("utf-8"),
            "depth_b64": base64.b64encode(depth_bytes).decode("utf-8"),
            "objects": objects,
        }
        return result_dict

    def _send_result(self, conn, result_dict):
        """将推理结果打包成下行帧（topic_id=100）发送给广播器。"""
        payload = json.dumps(result_dict).encode("utf-8")
        self._result_seq += 1
        header = struct.pack(
            HEADER_STRUCT, MAGIC, VERSION, DIR_DOWN, TOPIC_RESULT,
            self._result_seq, result_dict["stamp"], len(payload)
        )
        # 发送加锁，防止多线程推理时帧交错
        with self._send_lock:
            try:
                conn.sendall(header + payload)
            except OSError as e:
                # socket 断开是必须处理的异常边界
                print(f"[YoloRealTcpService] 发送结果失败: {e}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="YOLO 实机版实时推理服务（纯 Python TCP server，非 ROS）")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=9010, help="监听端口（默认 9010）")
    parser.add_argument("--prompt_model_path", default="./yoloe-v8m-seg2.pt",
                        help="YOLO 模型路径（含 .engine 则自动启用 TensorRT）")
    parser.add_argument("--prompt_file", default="./prompt/prompt.txt",
                        help="prompt 类别文本文件路径")
    parser.add_argument("--clip_model_type", default="mobileclip_b",
                        help="MobileCLIP 模型类型")
    parser.add_argument("--clip_model_path", default="./mobileclip_blt.pt",
                        help="MobileCLIP 权重路径")
    parser.add_argument("--use_prompt", action="store_true", default=True,
                        help="启用 prompt 模式（默认开启，可用 --no_prompt 关闭）")
    parser.add_argument("--no_prompt", action="store_false", dest="use_prompt",
                        help="关闭 prompt 模式")
    parser.add_argument("--use_clip", action="store_true", default=True,
                        help="启用 MobileCLIP 文本编码（默认开启，可用 --no_clip 关闭）")
    parser.add_argument("--no_clip", action="store_false", dest="use_clip",
                        help="关闭 MobileCLIP 文本编码")
    parser.add_argument("--use_tensorrt", action="store_true", default=None,
                        help="显式启用 TensorRT（默认 None 为自动判断：路径含 .engine）")
    parser.add_argument("--no_tensorrt", action="store_false", dest="use_tensorrt",
                        help="强制关闭 TensorRT")
    parser.add_argument("--export_mode", action="store_true", default=True,
                        help="启动时导出 TensorRT engine（默认开启，可用 --no_export_mode 关闭）")
    parser.add_argument("--no_export_mode", action="store_false", dest="export_mode",
                        help="关闭 engine 导出")
    parser.add_argument("--resize_width", type=int, default=640,
                        help="推理前 RGB 缩放宽度（默认 640）")
    parser.add_argument("--imgsz_h", type=int, default=480,
                        help="推理 / 导出固定输入高（默认 480）")
    parser.add_argument("--imgsz_w", type=int, default=640,
                        help="推理 / 导出固定输入宽（默认 640）")
    parser.add_argument("--conf", type=float, default=0.4,
                        help="model.predict 置信度阈值（默认 0.4）")
    parser.add_argument("--time_slop", type=float, default=0.005,
                        help="RGB-D 时间同步容差，秒（默认 0.005）")
    parser.add_argument("--odom_sync_slop", type=float, default=0.005,
                        help="odom 插值最大允许时间差，秒（默认 0.005）")
    parser.add_argument("--visualize", action="store_true", default=False,
                        help="显示可视化窗口（默认关闭，可用 --no_visualize 关闭）")
    parser.add_argument("--no_visualize", action="store_false", dest="visualize",
                        help="关闭可视化")
    parser.add_argument("--visualize_scale", type=float, default=0.5,
                        help="可视化缩放比例（默认 0.5）")
    parser.add_argument("--debug", action="store_true", default=False,
                        help="打印调试信息")
    args = parser.parse_args()

    service = YoloRealTcpService(
        host=args.host,
        port=args.port,
        prompt_model_path=args.prompt_model_path,
        prompt_file=args.prompt_file,
        clip_model_type=args.clip_model_type,
        clip_model_path=args.clip_model_path,
        use_prompt=args.use_prompt,
        use_clip=args.use_clip,
        export_mode=args.export_mode,
        use_tensorrt=args.use_tensorrt,
        resize_width=args.resize_width,
        imgsz_h=args.imgsz_h,
        imgsz_w=args.imgsz_w,
        conf=args.conf,
        time_slop=args.time_slop,
        odom_sync_slop=args.odom_sync_slop,
        visualize=args.visualize,
        visualize_scale=args.visualize_scale,
        debug=args.debug,
    )
    service.start()
