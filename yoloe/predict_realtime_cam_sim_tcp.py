#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
predict_realtime_cam_sim_tcp.py
===============================
YOLOE 仿真版实时推理服务（纯 Python、非 ROS 外部进程，机载运行）。

是 ros_ws/uss-nav/yoloe/predict_realtime_cam_sim.py 的 TCP 解耦版：
- 本脚本作为 TCP server，接收广播器（C++ ROS 节点）通过 TCP 发来的
  rgb / depth / odom 三路帧数据；
- 内部完成与 ROS 仿真版一致的 YOLOE prompt 模型推理 + MobileCLIP 文本编码；
- 推理结果（原始 rgb/depth 压缩字节 + 各目标 mask PNG / 512 维 word_vector）
  通过同一 TCP 连接以 JSON 形式回发给广播器，由广播器重建 EncodeMask 消息。

本脚本不依赖 rospy / cv_bridge，仅依赖 opencv / numpy / torch /
ultralytics(YOLOE) / mobileclip / base64 / json / struct / socket / threading。

帧协议（与广播器 C++ 端共享，全部大端序 / network order，与
yoloe_remote_service.py 完全一致）：
    头部固定 4+1+1+2+8+8+4 = 28 字节：
        magic     : 4 字节  b"TLBC"
        version   : 1 字节  1
        direction : 1 字节  0=上行(广播器->本服务), 1=下行(本服务->广播器)
        topic_id  : 2 字节  uint16 大端: 1=rgb, 2=depth, 3=odom,
                   50=组合帧(rgb+depth+odom 打包), 100=EncodeMask 结果
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

import cv2
import numpy as np
import torch
from ultralytics import YOLO, YOLOE

# ------------------------------ 帧协议常量 ------------------------------ #
MAGIC = b"TLBC"               # 帧魔数，用于识别帧起始
VERSION = 1                   # 协议版本号
DIR_UP = 0                    # 上行：广播器 -> 本服务
DIR_DOWN = 1                  # 下行：本服务 -> 广播器
TOPIC_RGB = 1                 # 上行：rgb 压缩图（jpeg）
TOPIC_DEPTH = 2               # 上行：depth 压缩图（png）
TOPIC_ODOM = 3                # 上行：odom JSON
TOPIC_PACKED = 50             # 上行：组合帧（rgb+depth+odom 打包，广播器已同步）
TOPIC_RESULT = 100            # 下行：EncodeMask 结果 JSON
HEADER_STRUCT = ">4sBBHQdI"   # 帧头打包格式：magic(4s) version(B) direction(B) topic(H) seq(Q) stamp(d) payload_len(I)
HEADER_SIZE = struct.calcsize(HEADER_STRUCT)  # 28 字节


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


class YoloeSimTcpService:
    """YOLOE 仿真版 TCP 推理服务。

    作为 TCP server 接收广播器上行帧（rgb/depth/odom），rgb 作为触发帧，
    depth/odom 与 rgb 的 stamp 差在 time_slop 内即触发推理线程，结果通过
    下行帧（topic_id=100, JSON）回发给广播器。
    """

    def __init__(self, host="127.0.0.1", port=9010,
                 model_path="./prompt/yoloe_pretrain/yoloe-11m-seg-pf.pt",
                 prompt_model_path="./prompt/yoloe_pretrain/yoloe-11m-seg.pt",
                 prompt_file="./prompt/prompt.txt",
                 clip_model_type="mobileclip_b",
                 clip_model_path="./mobileclip_blt.pt",
                 use_prompt=True, use_clip=True,
                 conf=0.4, resize_width=640, time_slop=0.1,
                 device="cuda:0", debug=False):
        # ---- 网络与推理参数 ----
        self.host = host
        self.port = port
        self.conf = conf                    # model.predict 的置信度阈值
        self.resize_width = resize_width    # 推理前将 RGB 宽度缩放到该值
        self.time_slop = time_slop          # 三路帧时间同步容差（秒）
        self.device = device
        self.debug = debug

        # ---- 模型路径与开关（默认值与 ROS 仿真版一致） ----
        self.model_path = model_path
        self.prompt_model_path = prompt_model_path
        self.prompt_file = prompt_file
        self.clip_model_type = clip_model_type
        self.clip_model_path = clip_model_path
        self.use_prompt = use_prompt
        self.use_clip = use_clip

        # ---- 最近一帧的三路数据缓存（用于时间同步判断与推理触发） ----
        self._latest_rgb = None    # {"stamp", "bgr", "payload"}
        self._latest_depth = None  # {"stamp", "array", "payload"}
        self._latest_odom = None   # {"stamp", "arr"}

        # ---- 并发控制 ----
        self._infer_lock = threading.Lock()   # 保证同一时刻只有一个推理线程
        self._send_lock = threading.Lock()    # 下行发送加锁，防止多线程交错写 socket
        self._result_seq = 0                  # 下行 EncodeMask 帧的 seq（独立自增）
        self._conn = None                     # 当前连接句柄（单连接模式）

        # 模型加载：失败则打印清晰错误并退出（无模型时服务无意义）
        try:
            self._load_models()
        except Exception as e:
            print(f"[YoloeSimTcpService] 模型加载失败: {e}")
            sys.exit(1)

    # ------------------------------------------------------------------ #
    # 模型加载（逻辑照搬 ROS 仿真版 predict_realtime_cam_sim.py 的 __init__）
    # 增强：使用 YOLO() 自动分发，同时兼容 YOLOE（含 yoloe-26）与纯 YOLO26 权重。
    # ------------------------------------------------------------------ #
    def _load_models(self):
        if not torch.cuda.is_available():
            print("[YoloeSimTcpService] CUDA 不可用，无法运行 YOLOE 推理")
            sys.exit(1)

        # Prompt 相关初始化：先用未融合模型根据 prompt 文本计算 vocab
        # 注意：YOLO() 会按文件名自动分发——文件名含 "yoloe" 时返回 YOLOE 类实例，
        # 纯 YOLO26（如 yolo26n-seg.pt）返回标准 YOLO 类实例。
        if self.use_prompt:
            with open(self.prompt_file, "r") as f:
                self.names = [x.strip() for x in f.readlines()]

            # 自动分发加载 prompt 模型，判断是否为 YOLOE（开放词汇）模型
            prompt_model = YOLO(self.prompt_model_path)
            if isinstance(prompt_model, YOLOE):
                # ---- YOLOE（含 yoloe-26）开放词汇流程：计算 vocab 并注入 ----
                print("[YoloeSimTcpService] 检测到 YOLOE 模型，启用开放词汇注入")
                unfused_model = prompt_model
                unfused_model.eval()
                unfused_model.cuda()
                self.vocab = unfused_model.get_vocab(self.names)
                print(f"[YoloeSimTcpService] vocab 计算完成，共 {len(self.names)} 个类别")

                self.model = YOLO(self.prompt_model_path).cuda()
                self.model.set_vocab(self.vocab, names=self.names)
                # 原实现将检测头设置为融合推理模式，并放宽内部 conf / max_det
                self.model.model.model[-1].is_fused = True
                self.model.model.model[-1].conf = 0.001
                self.model.model.model[-1].max_det = 1000
            else:
                # ---- 纯 YOLO26（闭集）流程：跳过 vocab 注入，使用模型自带类别 ----
                print("[YoloeSimTcpService] 检测到纯 YOLO26 模型，跳过开放词汇注入"
                      "（使用模型自带类别，prompt 文件不生效）")
                self.model = prompt_model.cuda()
        else:
            # use_prompt=False：同样自动分发，纯 YOLO26 与 YOLOE 均可加载
            self.model = YOLO(self.model_path).cuda()
        self.model.eval()

        # MobileCLIP 文本编码器（生成 512 维 word_vector）
        # 惰性 import：仅启用 use_clip 时加载 mobileclip，纯 YOLO26 闭集测试可跳过
        if self.use_clip:
            import mobileclip
            self.clip_model, _, self.clip_preprocess = \
                mobileclip.create_model_and_transforms(self.clip_model_type,
                                                       pretrained=self.clip_model_path)
            self.clip_model.to("cuda")
            self.clip_model.eval()
            self.clip_tokenizer = mobileclip.get_tokenizer(self.clip_model_type)

        # 模型预热（照搬 ROS 仿真版的 model_heat）
        self._model_heat()

        print("[YoloeSimTcpService] 模型初始化完成")

    def _model_heat(self):
        """模型预热：用示例图跑推理并统计 FPS，CLIP 同样预热（与 ROS 仿真版一致）。"""
        img_input = cv2.imread('./ultralytics/assets/bus.jpg')
        if img_input is None:
            print("[YoloeSimTcpService] 预热图 ./ultralytics/assets/bus.jpg 不存在，跳过模型预热")
            return
        for _ in range(2):
            time_start = time.perf_counter()
            with torch.no_grad():
                for _ in range(50):
                    self.model.predict(img_input, conf=0.3, device=self.device,
                                       save=False, verbose=False)
            time_cost = time.perf_counter() - time_start
            fps = 100 / time_cost
            print(f"[YoloeSimTcpService] YOLOE 预热 FPS: {fps:.2f}")

        if self.use_clip:
            text = self.clip_tokenizer(["a diagram", "a dog", "a cat", "bridge", "office chair", "chair"]).to(self.device)
            with torch.no_grad():
                for _ in range(2):
                    time_start = time.perf_counter()
                    for _ in range(50):
                        self.clip_model.encode_text(text)
                    time_cost = time.perf_counter() - time_start
                    fps = 100 / time_cost
                    print(f"[YoloeSimTcpService] CLIP 预热 FPS: {fps:.2f}")

    # ------------------------------------------------------------------ #
    # TCP server 主流程
    # ------------------------------------------------------------------ #
    def start(self):
        """创建 TCP server 并循环接受广播器连接（单连接模式）。"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[YoloeSimTcpService] 开始监听 {self.host}:{self.port} (time_slop={self.time_slop}s)")
        try:
            while True:
                conn, addr = server.accept()
                print(f"[YoloeSimTcpService] 广播器已连接: {addr}")
                # 单连接模式：若上一连接仍有残留，先关闭再处理新连接
                if self._conn is not None:
                    try:
                        self._conn.close()
                    except OSError:
                        pass
                self._conn = conn
                self._handle_connection(conn)
                print(f"[YoloeSimTcpService] 广播器连接断开: {addr}")
        except KeyboardInterrupt:
            # 仅处理 Ctrl-C 优雅退出，不吞掉其他异常
            print("[YoloeSimTcpService] 收到 Ctrl-C，服务退出")
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
                print(f"[YoloeSimTcpService] recv 出错，连接终止: {e}")
                break
            if not data:
                # 对端关闭连接
                print("[YoloeSimTcpService] 对端关闭连接")
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

        if topic_id == TOPIC_PACKED:
            # 组合帧：rgb/depth/odom 已由广播器同步打包，直接解包并触发推理
            # payload 布局（全部大端）：u32 rgb_len + rgb(jpeg) + u32 depth_len
            # + depth(png) + u32 odom_len + odom_json(UTF-8)
            try:
                off = 0
                rgb_len = struct.unpack_from(">I", frame["payload"], off)[0]; off += 4
                rgb_bytes = frame["payload"][off:off+rgb_len]; off += rgb_len
                depth_len = struct.unpack_from(">I", frame["payload"], off)[0]; off += 4
                depth_bytes = frame["payload"][off:off+depth_len]; off += depth_len
                odom_len = struct.unpack_from(">I", frame["payload"], off)[0]; off += 4
                odom_json_bytes = frame["payload"][off:off+odom_len]
            except (struct.error, IndexError) as e:
                # 长度字段越界 / 数据不完整属协议解析边界，必须捕获
                print(f"[YoloeSimTcpService][debug] 组合帧解析失败: {e}")
                return
            # rgb 解码（jpeg 压缩字节 -> BGR 图）
            bgr = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                print("[YoloeSimTcpService] rgb jpeg 解码失败，丢弃该帧")
                return
            # depth 解码（png 压缩字节 -> 仿真深度图 uchar/uint16）
            depth = cv2.imdecode(np.frombuffer(depth_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
            if depth is None:
                print("[YoloeSimTcpService] depth png 解码失败，丢弃该帧")
                return
            # odom 解析（UTF-8 JSON，字段缺省时回退默认值）
            try:
                odom = json.loads(odom_json_bytes.decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                print(f"[YoloeSimTcpService] odom JSON 解析失败，丢弃该帧: {e}")
                return
            odom_arr = [float(odom.get(k, 0.0)) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
            odom_arr[6] = float(odom.get("qw", 1.0))  # qw 默认 1.0
            # 更新三路缓存（组合帧已由广播器同步打包，三路 stamp 相同）
            stamp = frame["stamp"]
            self._latest_rgb = {"stamp": stamp, "bgr": bgr, "payload": rgb_bytes}
            self._latest_depth = {"stamp": stamp, "array": depth, "payload": depth_bytes}
            self._latest_odom = {"stamp": stamp, "arr": odom_arr}
            # 组合帧已同步好：直接触发推理，无需再做三路时间同步校验
            self._trigger_infer_packed(conn)
            return

        if topic_id == TOPIC_RGB:
            # rgb：jpeg 压缩字节 -> cv2.imdecode 得到 BGR 图
            bgr = cv2.imdecode(np.frombuffer(frame["payload"], np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                print("[YoloeSimTcpService] rgb jpeg 解码失败，丢弃该帧")
                return
            self._latest_rgb = {
                "stamp": frame["stamp"],
                "bgr": bgr,
                "payload": frame["payload"],   # 保留原始压缩字节用于回发
            }
            # rgb 是触发帧：收到后尝试一次推理
            self._try_infer(conn)

        elif topic_id == TOPIC_DEPTH:
            # depth：png 压缩字节 -> cv2.imdecode(IMREAD_UNCHANGED)，
            # 得到仿真深度图（uchar 或 uint16）
            depth = cv2.imdecode(np.frombuffer(frame["payload"], np.uint8), cv2.IMREAD_UNCHANGED)
            if depth is None:
                print("[YoloeSimTcpService] depth png 解码失败，丢弃该帧")
                return
            self._latest_depth = {
                "stamp": frame["stamp"],
                "array": depth,
                "payload": frame["payload"],   # 保留原始压缩字节用于回发
            }

        elif topic_id == TOPIC_ODOM:
            # odom：UTF-8 JSON {"x":..,"y":..,"z":..,"qx":..,"qy":..,"qz":..,"qw":..}
            try:
                odom = json.loads(frame["payload"].decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                print(f"[YoloeSimTcpService] odom JSON 解析失败，丢弃该帧: {e}")
                return
            self._latest_odom = {
                "stamp": frame["stamp"],
                "arr": [
                    float(odom.get("x", 0.0)), float(odom.get("y", 0.0)), float(odom.get("z", 0.0)),
                    float(odom.get("qx", 0.0)), float(odom.get("qy", 0.0)), float(odom.get("qz", 0.0)),
                    float(odom.get("qw", 1.0)),
                ],
            }

        else:
            print(f"[YoloeSimTcpService] 未知 topic_id={topic_id}，丢弃该帧")

    # ------------------------------------------------------------------ #
    # 推理触发与执行
    # ------------------------------------------------------------------ #
    def _trigger_infer_packed(self, conn):
        """组合帧触发：三路已由广播器同步打包（stamp 相同），直接启动推理线程。

        与 _try_infer 的区别：跳过三路时间同步校验（组合帧必然满足），
        其余（推理锁、异步线程、回发）完全一致。
        """
        # 同一时刻只允许一个推理线程：上一帧推理未结束时跳过当前触发帧
        if not self._infer_lock.acquire(blocking=False):
            if self.debug:
                print("[YoloeSimTcpService][debug] 上一帧推理仍在进行，跳过当前组合帧")
            return

        rgb = self._latest_rgb
        depth = self._latest_depth
        odom = self._latest_odom
        # 异步推理：避免阻塞收帧循环（daemon 线程）
        threading.Thread(target=self._run_infer, args=(conn, rgb, depth, odom), daemon=True).start()

    def _try_infer(self, conn):
        """rgb 触发帧处理：检查三路时间同步后，异步启动推理线程。"""
        if self._latest_rgb is None or self._latest_depth is None or self._latest_odom is None:
            # 三路数据尚未齐备，等待后续帧
            return

        rgb_stamp = self._latest_rgb["stamp"]
        depth_stamp = self._latest_depth["stamp"]
        odom_stamp = self._latest_odom["stamp"]
        # 时间同步校验：depth / odom 与 rgb 的 stamp 差需在 time_slop 内
        if abs(depth_stamp - rgb_stamp) > self.time_slop or abs(odom_stamp - rgb_stamp) > self.time_slop:
            if self.debug:
                print(f"[YoloeSimTcpService][debug] 等待同步: rgb={rgb_stamp:.3f} "
                      f"depth={depth_stamp:.3f} odom={odom_stamp:.3f}")
            return

        # 同一时刻只允许一个推理线程：上一帧推理未结束时跳过当前触发帧
        if not self._infer_lock.acquire(blocking=False):
            if self.debug:
                print("[YoloeSimTcpService][debug] 上一帧推理仍在进行，跳过当前触发帧")
            return

        rgb = self._latest_rgb
        depth = self._latest_depth
        odom = self._latest_odom
        # 异步推理：避免阻塞收帧循环（daemon 线程）
        threading.Thread(target=self._run_infer, args=(conn, rgb, depth, odom), daemon=True).start()

    def _run_infer(self, conn, rgb, depth, odom):
        """推理线程入口：执行推理并在有结果时回发，最后释放推理锁。"""
        try:
            result = self._infer(rgb["bgr"], depth["array"], odom["arr"], rgb["stamp"],
                                 rgb["payload"], depth["payload"])
            if result is not None:
                self._send_result(conn, result)
        except Exception as e:
            # 推理线程内的兜底异常打印，避免线程静默死亡
            print(f"[YoloeSimTcpService] 推理线程异常: {e}")
        finally:
            self._infer_lock.release()

    def _infer(self, rgb_bgr, depth, odom, stamp, rgb_bytes, depth_bytes):
        """核心推理（逻辑照搬 ROS 仿真版 processing_loop 的推理部分）。

        参数:
            rgb_bgr: BGR 图像（由 jpeg 解码得到）
            depth  : 深度图数组（本推理中不参与计算，仅随签名保留）
            odom   : [x, y, z, qx, qy, qz, qw]
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

        # 2. YOLOE 推理
        t1 = time.perf_counter()
        with torch.no_grad():
            results = self.model.predict(
                cv_rgb_resized, conf=self.conf, device=self.device,
                verbose=False, save=False
            )
        if self.debug:
            print(f"[YoloeSimTcpService][debug] Predict time: {((time.perf_counter() - t1) * 1e3):.2f}ms")
        if not results or len(results) == 0:
            return None
        result = results[0]

        # 3. 提取标签 / 置信度
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

        # 4. 掩码提取：masks.data -> 二值图 -> PNG 压缩
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
                        print(f"[YoloeSimTcpService] 掩码 {i} PNG 编码失败")
                except Exception as e:
                    # 个别掩码转换失败不应中断整帧处理
                    print(f"[YoloeSimTcpService] 掩码 {i} 编码异常: {e}")

        # 5. MobileCLIP 文本编码：tokenizer -> encode_text -> 归一化 -> float64[512]
        if self.use_clip:
            try:
                text = self.clip_tokenizer(labels).to(self.device)
                with torch.no_grad():
                    text_features = self.clip_model.encode_text(text)
                    text_features /= text_features.norm(dim=-1, keepdim=True)
                    text_features = text_features.cpu().numpy()
                for i in range(text_features.shape[0]):
                    objects[i]["word_vector"] = text_features[i].astype(np.float64).tolist()
            except Exception as e:
                print(f"[YoloeSimTcpService] CLIP 编码异常: {e}")

        # 6. 检测展示图：yolo plot（检测框+掩码+置信度+标签），RGB->BGR 后 JPEG 编码
        #    生成失败不影响主结果：仅打印日志并省略该字段
        vis_b64 = ""
        try:
            vis_img = result.plot(boxes=True, masks=True, conf=True, labels=True)
            vis_img = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
            ok, vis_buf = cv2.imencode(".jpg", vis_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                vis_b64 = base64.b64encode(vis_buf.tobytes()).decode("ascii")
        except Exception as e:
            print(f"[YoloeSimTcpService] 检测展示图生成失败: {e}")

        # 7. 组装回发 JSON（rgb/depth 使用收到的原始压缩字节，不重编码）
        result_dict = {
            "stamp": stamp,
            "odom": [float(v) for v in odom],
            "rgb_b64": base64.b64encode(rgb_bytes).decode("utf-8"),
            "depth_b64": base64.b64encode(depth_bytes).decode("utf-8"),
            "objects": objects,
            "vis_b64": vis_b64,
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
                print(f"[YoloeSimTcpService] 发送结果失败: {e}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="YOLOE 仿真版实时推理服务（纯 Python TCP server，非 ROS）")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=9010, help="监听端口（默认 9010）")
    parser.add_argument("--model_path", default="./prompt/yoloe_pretrain/yoloe-11m-seg-pf.pt",
                        help="非 prompt 模式的 YOLOE 模型路径")
    parser.add_argument("--prompt_model_path", default="./prompt/yoloe_pretrain/yoloe-11m-seg.pt",
                        help="prompt 模式的 YOLOE 模型路径")
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
    parser.add_argument("--conf", type=float, default=0.4,
                        help="model.predict 置信度阈值（默认 0.4）")
    parser.add_argument("--resize_width", type=int, default=640,
                        help="推理前 RGB 缩放宽度（默认 640）")
    parser.add_argument("--time_slop", type=float, default=0.1,
                        help="三路帧时间同步容差，秒（默认 0.01）")
    parser.add_argument("--debug", action="store_true", default=False,
                        help="打印调试信息")
    args = parser.parse_args()

    service = YoloeSimTcpService(
        host=args.host,
        port=args.port,
        model_path=args.model_path,
        prompt_model_path=args.prompt_model_path,
        prompt_file=args.prompt_file,
        clip_model_type=args.clip_model_type,
        clip_model_path=args.clip_model_path,
        use_prompt=args.use_prompt,
        use_clip=args.use_clip,
        conf=args.conf,
        resize_width=args.resize_width,
        time_slop=args.time_slop,
        debug=args.debug,
    )
    service.start()
