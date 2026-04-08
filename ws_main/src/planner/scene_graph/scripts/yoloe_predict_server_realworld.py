import rospy
import cv2
import threading
from queue import Queue, Full
from collections import deque
import copy
import math

from ultralytics import YOLO, YOLOE
import mobileclip
import torch
import numpy as np

import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

try:
    from scene_graph.msg import EncodeMask
    from scene_graph.msg import WordVector
except ImportError:
    print("EncodeMask not found, please check if you have installed ROS")
    exit()

# --------------------------------------------------------------------------------
# 新增: 里程计缓冲与插值处理类
# --------------------------------------------------------------------------------
class OdometryBuffer:
    def __init__(self, max_len=500, sync_slop=0.1):
        """
        :param max_len: 缓存队列最大长度
        :param sync_slop: 最大允许的时间同步误差(秒)
        """
        self.lock = threading.Lock()
        self.buffer = deque(maxlen=max_len)
        self.sync_slop = sync_slop

    def add_odom(self, odom_msg):
        """将新的odom消息加入队列"""
        with self.lock:
            self.buffer.append(odom_msg)
            # 简单的清理逻辑：如果队头数据太老（比当前新数据早太多），也可以清理，
            # 但由于使用了maxlen的deque，自动覆盖旧数据，这里暂不需要手动清理。

    def get_interpolated_odom(self, target_time):
        """
        根据目标时间戳计算插值后的里程计
        :param target_time: rospy.Time 对象
        :return: 插值后的 nav_msgs/Odometry 对象，如果失败返回 None
        """
        target_sec = target_time.to_sec()

        with self.lock:
            if len(self.buffer) < 2:
                return None

            # 1. 快速检查时间范围
            newest_time = self.buffer[-1].header.stamp.to_sec()
            oldest_time = self.buffer[0].header.stamp.to_sec()

            # 如果目标时间太新或太旧，超出缓存范围且超过容差，则无法插值
            if target_sec > newest_time + self.sync_slop:
                return None
            if target_sec < oldest_time - self.sync_slop:
                return None

            # 2. 寻找前后帧 (Binary search would be faster O(logN), but linear O(N) is fine for small buffers)
            # 由于数据是按时间顺序的，且我们要找的是最新的匹配，从后往前找通常更快
            idx = -1
            for i in range(len(self.buffer) - 1, -1, -1):
                if self.buffer[i].header.stamp.to_sec() <= target_sec:
                    idx = i
                    break

            # Case A: 目标时间比最早的还早，但在容差范围内 -> 返回最早那帧
            if idx == -1:
                return copy.deepcopy(self.buffer[0])

            # Case B: 目标时间比最新的还晚，但在容差范围内 -> 返回最新那帧
            if idx == len(self.buffer) - 1:
                return copy.deepcopy(self.buffer[-1])

            # Case C: 正常插值范围
            odom_prev = self.buffer[idx]
            odom_next = self.buffer[idx + 1]

        # 3. 执行插值计算 (释放锁后计算，减少阻塞)
        return self._interpolate(odom_prev, odom_next, target_sec)

    def _interpolate(self, odom_prev, odom_next, target_sec):
        """执行位置线性插值和四元数球面插值(Slerp)"""
        t0 = odom_prev.header.stamp.to_sec()
        t1 = odom_next.header.stamp.to_sec()

        # 防止除以零
        if abs(t1 - t0) < 1e-9:
            return copy.deepcopy(odom_prev)

        alpha = (target_sec - t0) / (t1 - t0)

        res_odom = Odometry()
        # 复制 header (使用目标时间) 和 frame_id
        res_odom.header.stamp = rospy.Time.from_sec(target_sec)
        res_odom.header.frame_id = odom_prev.header.frame_id
        res_odom.child_frame_id = odom_prev.child_frame_id

        # --- 位置插值 (Linear) ---
        p0 = odom_prev.pose.pose.position
        p1 = odom_next.pose.pose.position
        res_odom.pose.pose.position.x = p0.x + alpha * (p1.x - p0.x)
        res_odom.pose.pose.position.y = p0.y + alpha * (p1.y - p0.y)
        res_odom.pose.pose.position.z = p0.z + alpha * (p1.z - p0.z)

        # --- 姿态插值 (Slerp) ---
        q0 = [odom_prev.pose.pose.orientation.x, odom_prev.pose.pose.orientation.y,
              odom_prev.pose.pose.orientation.z, odom_prev.pose.pose.orientation.w]
        q1 = [odom_next.pose.pose.orientation.x, odom_next.pose.pose.orientation.y,
              odom_next.pose.pose.orientation.z, odom_next.pose.pose.orientation.w]

        q_res = self._slerp(q0, q1, alpha)

        res_odom.pose.pose.orientation.x = q_res[0]
        res_odom.pose.pose.orientation.y = q_res[1]
        res_odom.pose.pose.orientation.z = q_res[2]
        res_odom.pose.pose.orientation.w = q_res[3]

        # 线速度和角速度也可以插值 (Linear)
        v0 = odom_prev.twist.twist
        v1 = odom_next.twist.twist
        res_odom.twist.twist.linear.x = v0.linear.x + alpha * (v1.linear.x - v0.linear.x)
        res_odom.twist.twist.linear.y = v0.linear.y + alpha * (v1.linear.y - v0.linear.y)
        res_odom.twist.twist.linear.z = v0.linear.z + alpha * (v1.linear.z - v0.linear.z)
        res_odom.twist.twist.angular.x = v0.angular.x + alpha * (v1.angular.x - v0.angular.x)
        res_odom.twist.twist.angular.y = v0.angular.y + alpha * (v1.angular.y - v0.angular.y)
        res_odom.twist.twist.angular.z = v0.angular.z + alpha * (v1.angular.z - v0.angular.z)

        return res_odom

    def _slerp(self, q0, q1, t):
        """四元数球面线性插值"""
        # 归一化
        q0 = np.array(q0)
        q1 = np.array(q1)
        q0 /= np.linalg.norm(q0)
        q1 /= np.linalg.norm(q1)

        dot = np.dot(q0, q1)

        # 如果点积为负，反转 q1 以走最短路径
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        # 如果太接近，直接线性插值避免数值不稳定
        if dot > 0.9995:
            result = q0 + t * (q1 - q0)
            return result / np.linalg.norm(result)

        theta_0 = np.arccos(dot)  # theta_0 = angle between input vectors
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q0) + (s1 * q1)


class YoloeDetectorNode:
    def __init__(self):
        # 参数配置
        self.model_path          = rospy.get_param('~model_path', './prompt/yoloe_pretrain/yoloe-11m-seg-pf.pt')
        self.prompt_model_path   = rospy.get_param('~prompt_model_path', './prompt/yoloe_pretrain/yoloe-11m-seg.engine')
        self.prompt_file_path     = rospy.get_param('~prompt_file_path', './prompt/prompt.txt')

        self.clip_model_type     = rospy.get_param('~clip_model_type','mobileclip_b')
        self.clip_model_path     = rospy.get_param('~clip_model_path', './mobileclip_blt.pt')

        # 三个输入话题参数
        self.rgb_in_topic        = rospy.get_param('~rgb_topic',   '/camera/color/image_raw/compressed')
        self.depth_in_topic      = rospy.get_param('~depth_topic', '/camera/aligned_depth_to_color/image_raw/compressedDepth')
        self.odom_in_topic       = rospy.get_param('~odom_topic',  '/ekf_quat/ekf_odom')

        self.rgb_predict_out_topic = rospy.get_param('~output_topic', '/yoloe/plot')
        self.encodemask_out_topic  = rospy.get_param('~result_output_topic', '/yoloe/encodemask')

        self.use_prompt          = rospy.get_param('~use_prompt', True)
        self.use_clip            = rospy.get_param('~use_clip', True)
        self.resize_width        = rospy.get_param('~resize_width', 640)
        self.queue_size          = rospy.get_param('~queue_size', 3)
        self.visualize           = rospy.get_param('~visualize', True)
        self.visualize_scale     = rospy.get_param('~visualize_scale', 0.5)
        self.debug               = rospy.get_param('~debug', False)
        self.time_slop           = rospy.get_param('~time_slop', 0.005) # RGB-D 同步容差

        self.export_mode         = rospy.get_param('~export_mode', False)

        # [修改] 新增参数：Odom同步的最大时间差容限
        self.odom_sync_slop      = rospy.get_param('~odom_sync_slop', 0.005)

        self.is_rgb_compressed   = 'compressed' in self.rgb_in_topic
        self.is_depth_compressed = 'compressed' in self.depth_in_topic or 'compressedDepth' in self.depth_in_topic

        # CLIP 缓存字典
        self.clip_cache = {}

        # [修改] 初始化 Odom Buffer
        self.odom_buffer = OdometryBuffer(max_len=1000, sync_slop=self.odom_sync_slop)

        rospy.loginfo(f"Model path  : {self.model_path}")
        rospy.loginfo(f"RGB Topic   : {self.rgb_in_topic}")
        rospy.loginfo(f"Depth Topic : {self.depth_in_topic}")
        rospy.loginfo(f"Odom Topic  : {self.odom_in_topic}")

        if not torch.cuda.is_available():
            exit("*** CUDA is not available ***")

        rospy.loginfo("[!!!] *** Model INIT ...")
        self.use_trnsorrt = False
        # 模型初始化
        if self.use_prompt:
            with open(self.prompt_file_path, 'r') as f:
                self.names = [x.strip() for x in f.readlines()]

            if '.engine' in self.prompt_model_path:
                self.use_trnsorrt = True
                print("********** USE TENSOR-RT **********\n")
                self.model = YOLO(self.prompt_model_path)
            else:
                self.model = YOLO(self.prompt_model_path).cuda()
                # self.model.set_vocab(self.vocab, names=self.names)
                self.model.set_classes(self.names, self.model.get_text_pe(self.names))
                # self.model.model.model[-1].is_fused = True
                # self.model.model.model[-1].conf = 0.001
                # self.model.model.model[-1].max_det = 1000
        else:
            self.model = YOLO(self.model_path).cuda()

        # self.model.eval()
        if self.export_mode:
            export_model = self.model.export(
                format="engine",
                imgsz=(480, 640),      # 显式指定 高, 宽
                dynamic=False,         # 【关键】强制关闭动态形状！
                half=True,             # 开启 FP16 加速
                simplify=True          # 简化 ONNX 结构
                # workspace=8          # 显存限制
            )
            print(f"Exported model path: {export_model}")

        if self.use_clip:
            self.clip_model, _, self.clip_preprocess = \
                mobileclip.create_model_and_transforms(self.clip_model_type, pretrained=self.clip_model_path)
            self.clip_model.to('cuda')
            self.clip_model.eval()
            self.clip_tockenizer = mobileclip.get_tokenizer(self.clip_model_type)

        self.model_heat()

        # ROS相关初始化
        self.cv_bridge  = CvBridge()
        # self.image_pub  = rospy.Publisher(self.rgb_predict_out_topic, Image, queue_size=2)
        self.final_img_vis_pub = rospy.Publisher(self.rgb_predict_out_topic + '/final_vis', Image, queue_size=2)
        self.result_pub = rospy.Publisher(self.encodemask_out_topic, EncodeMask, queue_size=2)
        self.synced_odom_pub = rospy.Publisher("/yoloe/synced_odom", Odometry, queue_size=2)

        # 创建数据队列 (只存 RGBD)
        self.rgbd_queue = Queue(maxsize=self.queue_size)
        self.processing_thread = threading.Thread(target=self.processing_loop)
        self.thread_running = False

        self.inference_count = 0
        self.inference100_time_start = rospy.Time.now()

        rospy.loginfo("YOLOE server init success...")

    def model_heat(self):
        # 模型预热 (保持不变)
        img_input = cv2.imread('./ultralytics/assets/bus.jpg')
        img_input = cv2.resize(img_input, (480, 640))

        for j in range(2):
            with torch.no_grad():
                for i in range(10):
                    results = self.model.predict(img_input, conf=0.3, imgsz=(480, 640), device="cuda:0",
                                                 save=False, verbose=False)
        if self.use_clip:
            text = self.clip_tockenizer(["a diagram", "a dog"]).to("cuda:0")
            with torch.no_grad():
                for j in range(20):
                    self.clip_model.encode_text(text)

    def start(self):
        """启动处理线程并初始化话题同步器"""
        self.thread_running = True
        self.processing_thread.start()

        # 初始化话题订阅器
        if 'compressed' in self.rgb_in_topic:
            rospy.loginfo("[!!!] *** Compressed RGB image subscriber")
            rgb_sub = message_filters.Subscriber(self.rgb_in_topic, CompressedImage)
        else:
            rgb_sub = message_filters.Subscriber(self.rgb_in_topic, Image)

        if self.is_depth_compressed:
            rospy.loginfo("[!!!] *** Compressed Depth image subscriber")
            depth_sub = message_filters.Subscriber(self.depth_in_topic, CompressedImage)
        else:
            depth_sub = message_filters.Subscriber(self.depth_in_topic, Image)

        # [修改] 里程计单独订阅，不进入 message_filters
        self.odom_sub = rospy.Subscriber(self.odom_in_topic, Odometry, self.odom_callback, queue_size=100)

        # [修改] 时间同步器只同步 RGB 和 Depth
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], # 移除 odom_sub
            queue_size = 20,
            slop=self.time_slop
        )
        self.ts.registerCallback(self.rgbd_synced_callback)

        rospy.loginfo("YOLOE server start success...")

    def stop(self):
        self.thread_running = False
        if self.processing_thread.is_alive():
            self.processing_thread.join()
        rospy.loginfo("YOLOE server stop success...")

    # [修改] 独立的回调函数处理高频 Odom 数据
    def odom_callback(self, odom_msg):
        self.odom_buffer.add_odom(odom_msg)

    # [修改] 回调函数只接收 RGB 和 Depth
    def rgbd_synced_callback(self, rgb_msg, depth_msg):
        """同步回调函数：接收同步后的RGB和Depth消息"""
        # rospy.logdebug("Sync RGB-D Time: Rgb {}".format(rgb_msg.header.stamp.to_sec()))

        try:
            self.rgbd_queue.put_nowait((rgb_msg, depth_msg))
        except Full:
            try:
                self.rgbd_queue.get_nowait()
                self.rgbd_queue.put_nowait((rgb_msg, depth_msg))
            except Full:
                rospy.logwarn("Cannot add new data, queue operation failed")

    def processing_loop(self):
        """处理循环"""
        while self.thread_running and not rospy.is_shutdown():
            try:
                # 1. 从队列获取同步的 RGB-D 数据
                (rgb_msg, depth_msg) = self.rgbd_queue.get(timeout=1.0)

                # 2. [关键修改] 手动获取同步并插值后的 Odometry
                # 以 RGB 图像的时间戳为基准
                img_timestamp = rgb_msg.header.stamp
                synced_odom = self.odom_buffer.get_interpolated_odom(img_timestamp)

                if synced_odom is None:
                    # 如果找不到对应的 Odom，根据需求可以选择跳过该帧或打印警告
                    rospy.logwarn_throttle(1.0, f"Odom sync failed for time {img_timestamp.to_sec():.2f}. Buffer may be empty or laggy.")
                    # 严格模式下可以 continue，如果允许无odom运行则可以给一个默认值，这里选择 continue 保证数据质量
                    continue

                    # ------------------ RGB 转换 (保持原有逻辑) ------------------
                rgb_compressed_msg = None
                cv_rgb = None
                try:
                    if self.is_rgb_compressed:
                        cv_rgb = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
                        rgb_compressed_msg = rgb_msg
                    else:
                        cv_rgb = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                    cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
                except CvBridgeError as e:
                    rospy.logerr(f"RGB conversion error: {e}")
                    continue

                # ------------------ Depth 转换 (保持原有逻辑) ------------------
                depth_compressed_msg = None
                cv_depth = None
                try:
                    if self.is_depth_compressed:
                        depth_fmt, compress_fmt = depth_msg.format.split(';')
                        depth_fmt = depth_fmt.strip()
                        compress_fmt = compress_fmt.strip()
                        raw_data = np.frombuffer(depth_msg.data, np.uint8)

                        if 'compressedDepth' in compress_fmt:
                            if len(raw_data) > 12:
                                clean_data = raw_data[12:]
                                cv_depth = cv2.imdecode(clean_data, cv2.IMREAD_UNCHANGED)
                            else:
                                continue
                        else:
                            cv_depth = cv2.imdecode(raw_data, cv2.IMREAD_UNCHANGED)
                        depth_compressed_msg = depth_msg
                    else:
                        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                        depth_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(cv_depth, dst_format='png')
                except Exception as e:
                    rospy.logerr(f"Depth conversion error: {e}")
                    continue

                if cv_depth is None: continue

                # Resize Logic (保持不变)
                h, w, _ = cv_rgb.shape
                if w != self.resize_width:
                    cv_rgb_resized = cv2.resize(cv_rgb, (self.resize_width, int(h * self.resize_width / w)))
                else:
                    cv_rgb_resized = cv_rgb

            except Exception as e:
                rospy.logerr(f"Data preparation error: {e}")
                continue

            # >>>>>>>>>>>>>>>>>> 推理处理 >>>>>>>>>>>>>>>>>>> #
            encode_mask_msg = EncodeMask()
            encode_mask_msg.header.stamp  = rgb_msg.header.stamp
            # [修改] 使用插值后的 Odom
            encode_mask_msg.current_odom  = synced_odom
            encode_mask_msg.current_depth = depth_compressed_msg
            encode_mask_msg.current_rgb   = rgb_compressed_msg


            if self.debug:
                print("Sync Stamp RGB({}), Depth({}), Odom({})" \
                      .format(rgb_msg.header.stamp, depth_msg.header.stamp, synced_odom.header.stamp))

            # FPS 计算 (保持不变)
            if self.inference_count % 60 == 0 and self.inference_count > 0:
                self.inference_count = 0
                time_cost = (rospy.Time.now() - self.inference100_time_start).to_sec()
                self.inference100_time_start = rospy.Time.now()
                fps = 60 / time_cost
                rospy.loginfo(f"FPS: {fps:.2f}")

            # 模型推理
            t_yoloe_start = rospy.Time.now()
            with torch.no_grad():
                results = self.model.predict(
                    cv_rgb_resized, conf=0.4, imgsz=(480, 640), device="cuda:0",
                    verbose=False, save=False
                )
            t_vis_start = rospy.Time.now()


            # 可视化处理
            if self.visualize:
                # ... (原代码的可视化逻辑)
                try:
                    yolo_plot_img = results[0].plot(boxes=True, masks=True, conf=True, labels=True)
                    vis_yolo = cv2.cvtColor(yolo_plot_img, cv2.COLOR_RGB2BGR)
                    vis_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)
                    vis_depth_temp = cv_depth.copy()
                    vis_depth_temp[vis_depth_temp > 10000] = 10000
                    vis_depth_norm = cv2.normalize(vis_depth_temp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    vis_depth = cv2.applyColorMap(vis_depth_norm, cv2.COLORMAP_JET)

                    target_h, target_w = vis_yolo.shape[:2]
                    if vis_rgb.shape[:2] != (target_h, target_w): vis_rgb = cv2.resize(vis_rgb, (target_w, target_h))
                    if vis_depth.shape[:2] != (target_h, target_w): vis_depth = cv2.resize(vis_depth, (target_w, target_h))

                    combined_img = np.hstack((vis_yolo, vis_rgb, vis_depth))
                    if self.visualize_scale != 1.0:
                        new_w = int(combined_img.shape[1] * self.visualize_scale)
                        new_h = int(combined_img.shape[0] * self.visualize_scale)
                        final_vis_img = cv2.resize(combined_img, (new_w, new_h))
                    else:
                        final_vis_img = combined_img

                    self.final_img_vis_pub.publish(self.cv_bridge.cv2_to_imgmsg(final_vis_img, encoding="bgr8"))
                    self.synced_odom_pub.publish(synced_odom)
                except Exception as e:
                    pass


            t_clip_process_start = rospy.Time.now()
            # 结果打包 (保持不变)
            if results and len(results) > 0:
                predict_success = True
                result = results[0]
                label_results = []
                if hasattr(result, 'boxes') and result.boxes is not None:
                    class_ids = result.boxes.cls.cpu().numpy().astype(int)
                    confidences = result.boxes.conf.cpu().numpy()
                    class_names = [result.names[cls_id] for cls_id in class_ids] if \
                        hasattr(result, 'names') else [f"class_{cls_id}" for cls_id in class_ids]

                    for cls_name, cls_conf in zip(class_names, confidences):
                        label_results.append(f"{cls_name}")
                        encode_mask_msg.labels.append(cls_name)
                        encode_mask_msg.confs.append(cls_conf)

                if (len(label_results) == 0):
                    predict_success = False
                else:
                    if hasattr(result, 'masks') and result.masks is not None:
                        mask_arrays = result.masks.data.cpu().numpy()
                        for i in range(len(mask_arrays)):
                            single_mask = mask_arrays[i]
                            binary_mask = (single_mask * 255).astype(np.uint8)
                            try:
                                compressed_mask_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                                    binary_mask, dst_format='png;params=[16,0]'
                                )
                                compressed_mask_msg.format = "png"
                                encode_mask_msg.masks.append(compressed_mask_msg)
                            except Exception: pass

                    if self.use_clip:
                        try:
                            unknown_words = list(set([label for label in label_results if label not in self.clip_cache]))
                            if len(unknown_words) > 0:
                                text_input = self.clip_tockenizer(unknown_words).to("cuda:0")
                                with torch.no_grad():
                                    new_features = self.clip_model.encode_text(text_input)
                                    new_features /= new_features.norm(dim=-1, keepdim=True)
                                    new_features = new_features.cpu().numpy()
                                for idx, word in enumerate(unknown_words):
                                    self.clip_cache[word] = new_features[idx]
                            for label in label_results:
                                if label in self.clip_cache:
                                    feature_vec = self.clip_cache[label]
                                    word_vector_msg = WordVector()
                                    word_vector_msg.word_vector = feature_vec.astype(np.float64)
                                    encode_mask_msg.word_vectors.append(word_vector_msg)
                        except Exception as e:
                            predict_success = False
                            rospy.logerr(f"CLIP error: {e}")

                if predict_success and len(label_results) != 0:
                    self.result_pub.publish(encode_mask_msg)

            self.inference_count += 1
            t3 = rospy.Time.now()

            if self.debug:
                rospy.logwarn(f"YoloE time: {((t_clip_process_start - t_yoloe_start).to_sec() * 1e3):.2f}ms \n \
                                clip time: {((t3 - t_clip_process_start).to_sec() * 1e3):.2f}ms\n \
                                vis time: {((t_clip_process_start - t_vis_start).to_sec() * 1e3):.2f}ms")

if __name__ == '__main__':
    rospy.init_node('yoloe_detector_node', anonymous=True)
    try:
        node = YoloeDetectorNode()
        node.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.stop()
        cv2.destroyAllWindows()