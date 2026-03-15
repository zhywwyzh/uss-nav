import rospy
import cv2
import threading
from queue import Queue, Full
from ultralytics import YOLO, YOLOE
import mobileclip
import torch
import numpy as np

import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry

try:
    from scene_graph.msg import EncodeMask
    from scene_graph.msg import WordVector
except ImportError:
    print("EncodeMask not found, please check if you have installed ROS")
    exit()

class YoloeDetectorNode:
    def __init__(self):
        # 参数配置 - 新增深度图和Odometry话题参数
        self.model_path          = rospy.get_param('~model_path', './prompt/yoloe_pretrain/yoloe-11m-seg-pf.pt')
        self.prompt_model_path   = rospy.get_param('~prompt_model_path', './prompt/yoloe_pretrain/yoloe-11m-seg.pt')
        self.prompt_file_path    = rospy.get_param('~prompt_file_path', './prompt/prompt.txt')

        self.clip_model_type     = rospy.get_param('~clip_model_type','mobileclip_b')
        self.clip_model_path     = rospy.get_param('~clip_model_path', './mobileclip_blt.pt')
        
        # 三个输入话题参数
        self.rgb_in_topic        = rospy.get_param('~rgb_topic',   '/camera1/color/image/compressed')
        self.depth_in_topic      = rospy.get_param('~depth_topic', '/camera1/depth/image/compressed')
        self.odom_in_topic       = rospy.get_param('~odom_topic',  '/unity_odom')
        
        self.rgb_predict_out_topic = rospy.get_param('~output_topic', '/yoloe/plot')
        self.encodemask_out_topic  = rospy.get_param('~result_output_topic', '/yoloe/encodemask')
        
        self.use_prompt          = rospy.get_param('~use_prompt', True)
        self.use_clip            = rospy.get_param('~use_clip', True)
        self.resize_width        = rospy.get_param('~resize_width', 640)
        self.queue_size          = rospy.get_param('~queue_size', 10)
        self.visualize           = rospy.get_param('~visualize', False)
        self.debug               = rospy.get_param('~debug', False)
        self.time_slop           = rospy.get_param('~time_slop', 0.01)

        self.is_rgb_compressed   = 'compressed' in self.rgb_in_topic
        self.is_depth_compressed = 'compressed' in self.depth_in_topic


        rospy.loginfo(f"Model path  : {self.model_path}")
        rospy.loginfo(f"RGB Topic   : {self.rgb_in_topic}")
        rospy.loginfo(f"Depth Topic : {self.depth_in_topic}")
        rospy.loginfo(f"Odom Topic  : {self.odom_in_topic}")
        rospy.loginfo(f"Output Topic: {self.rgb_predict_out_topic}")

        if not torch.cuda.is_available():
            exit("*** CUDA is not available ***")

        # Prompt相关初始化
        if self.use_prompt:
            rospy.loginfo("[!!!] *** Prompt enabled, load prompt model and vocab")
            unfused_model = YOLOE(self.prompt_model_path)
            unfused_model.load(self.prompt_model_path)
            unfused_model.eval()
            unfused_model.cuda()
            with open(self.prompt_file_path, 'r') as f:
                self.names = [x.strip() for x in f.readlines()]
                self.vocab = unfused_model.get_vocab(self.names)
            # with open(self.prompt_file_path, 'r') as f:
            #     self.names = [x.strip() for x in f.readlines()]
            # print(self.names)
            # exit()
            rospy.loginfo("[!!!] *** Prompt enabled, load vocab success !")

        rospy.loginfo("[!!!] *** Model INIT ...")
        # 模型初始化
        if self.use_prompt:
            rospy.loginfo("[!!!] *** Text encode enabled, load prompt model and vocab")
            self.model = YOLOE(self.prompt_model_path).cuda()
            self.model.set_vocab(self.vocab, names=self.names)
            self.model.model.model[-1].is_fused = True
            self.model.model.model[-1].conf = 0.001
            self.model.model.model[-1].max_det = 1000
        else:
            self.model = YOLOE(self.model_path).cuda()
        self.model.eval()

        if self.use_clip:                                                          
            self.clip_model, _, self.clip_preprocess = \
               mobileclip.create_model_and_transforms(self.clip_model_type, pretrained=self.clip_model_path)
            self.clip_model.to('cuda')
            self.clip_model.eval()
            self.clip_tockenizer = mobileclip.get_tokenizer(self.clip_model_type)
            

        self.model_heat()
        
        # ROS相关初始化
        self.cv_bridge  = CvBridge()
        self.image_pub  = rospy.Publisher(self.rgb_predict_out_topic, Image, queue_size=2)
        self.result_pub = rospy.Publisher(self.encodemask_out_topic, EncodeMask, queue_size=2)
        
        # 创建数据队列(存储RGB、Depth、Odom联合数据)和线程控制变量
        self.data_queue = Queue(maxsize=self.queue_size)
        self.processing_thread = threading.Thread(target=self.processing_loop)
        self.thread_running = False

        self.inference_count = 0
        self.inference100_time_start = rospy.Time.now()

        rospy.loginfo("YOLOE server init success...")
    
    def model_heat(self):
        # 模型预热
        img_input = cv2.imread('./ultralytics/assets/bus.jpg')
        for j in range(2):
            time_start = rospy.Time.now()
            with torch.no_grad():
                for i in range(50):
                    results = self.model.predict(img_input, conf=0.3, device="cuda:0", 
                                                save=False, verbose=False)
                time_end = rospy.Time.now()
                time_cost = (time_end - time_start).to_sec()
                fps = 100 / time_cost
                rospy.loginfo(f"FPS: {fps:.2f}")
        
        if self.use_clip:
            text = self.clip_tockenizer(["a diagram", "a dog", "a cat", "bridge", "office chair", "chair"]).to("cuda:0")
            with torch.no_grad():
                for i in range(2):
                    time_start = rospy.Time.now()
                    for j in range(50):
                        text_features = self.clip_model.encode_text(text)
                    time_end = rospy.Time.now()
                    time_cost = (time_end - time_start).to_sec()
                    fps = 100 / time_cost
                    rospy.loginfo(f"CLIP FPS: {fps:.2f}")


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

        if 'compressed' in self.depth_in_topic:
            rospy.loginfo("[!!!] *** Compressed Depth image subscriber")
            depth_sub = message_filters.Subscriber(self.depth_in_topic, CompressedImage)
        else:
            depth_sub = message_filters.Subscriber(self.depth_in_topic, Image)

        odom_sub = message_filters.Subscriber(self.odom_in_topic, Odometry)

        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer( 
            [rgb_sub, depth_sub, odom_sub],
            queue_size = 50,
            slop=self.time_slop
        )
        self.ts.registerCallback(self.synced_callback)

        rospy.loginfo("YOLOE server start success...")

    def stop(self):
        """停止处理线程"""
        self.thread_running = False
        if self.processing_thread.is_alive():
            self.processing_thread.join()
        rospy.loginfo("YOLOE server stop success...")
    
    def add_synced_data(self, rgb_data, depth_data, odom_data):
        """将同步后的三话题数据加入队列"""
        try:
            self.data_queue.put_nowait((
                [rgb_data, depth_data, odom_data]
            ))
        except Full:
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait((
                    [rgb_data, depth_data, odom_data]
                ))
                rospy.logdebug("Queue is full, drop the oldest data")
            except Full:
                rospy.logwarn("Cannot add new data, queue operation failed")

    def synced_callback(self, rgb_msg, depth_msg, odom_msg):
        # 调试信息
        # print(f"Synced messages - RGB: {rgb_msg.header.stamp.to_sec():.8f}, " \
        #                 f"Depth: {depth_msg.header.stamp.to_sec():.8f}, " \
        #                 f"Odom: {odom_msg.header.stamp.to_sec():.8f}")
        # 将同步数据加入处理队列
        self.add_synced_data(
            rgb_msg, depth_msg, odom_msg
        )
    
        
    def processing_loop(self):
        """处理循环：从队列获取同步数据并处理"""
        while self.thread_running and not rospy.is_shutdown():
            try:
                # 从队列获取同步数据
                [rgb_msg, depth_msg, odom_msg] = self.data_queue.get(timeout=1.0)
                # 转换RGB图像
                rgb_compressed_msg = None
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
                
                # 转换深度图像
                depth_compressed_msg = None
                try:
                    if self.is_depth_compressed:
                        # cv_depth = self.cv_bridge.compressed_imgmsg_to_cv2(depth_msg, "passthrough")
                        depth_compressed_msg = depth_msg
                    else:
                        # 如果不是压缩msg则将depth信息压缩
                        depth_msg.encoding = "passthrough"
                        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                        depth_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(cv_depth, dst_format='png')
                        
                except CvBridgeError as e:
                    rospy.logerr(f"Depth conversion error: {e}")
                    continue
                
                # 图像缩放为统一大小，使得图像宽度为640
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
            # 使用RGB图像的时间戳作为基准时间戳
            encode_mask_msg.header.stamp  = rgb_msg.header.stamp
            encode_mask_msg.current_odom  = odom_msg
            encode_mask_msg.current_depth = depth_compressed_msg
            encode_mask_msg.current_rgb   = rgb_compressed_msg

            # # 这里使用opencv同时可视化depth_msg和rgb_msg
            # cv_depth = self.cv_bridge.compressed_imgmsg_to_cv2(depth_compressed_msg, "passthrough")
            # cv2.imshow("Depth", cv_depth)
            # cv2.imshow("RGB", cv_rgb_resized)
            # cv2.waitKey(1)

            # 计算FPS
            if self.inference_count % 60 == 0 and self.inference_count > 0:
                self.inference_count = 0
                time_cost = (rospy.Time.now() - self.inference100_time_start).to_sec()
                self.inference100_time_start = rospy.Time.now()
                fps = 60 / time_cost
                rospy.loginfo(f"FPS: {fps:.2f}")
            
            # 模型推理
            t1 = rospy.Time.now()
            with torch.no_grad():
                results = self.model.predict(
                    cv_rgb_resized, conf=0.4, device="cuda:0", 
                    verbose=self.debug, save=False
                )
            t2 = rospy.Time.now()

            # 提取检测结果并构建消息
            if results and len(results) > 0:
                predict_success = True
                result = results[0]
                label_results = []
                # 处理标签和置信度
                if hasattr(result, 'boxes') and result.boxes is not None:
                    class_ids = result.boxes.cls.cpu().numpy().astype(int)
                    confidences = result.boxes.conf.cpu().numpy()
                    class_names = [result.names[cls_id] for cls_id in class_ids] if \
                        hasattr(result, 'names') else [f"class_{cls_id}" for cls_id in class_ids]
                    
                    for cls_name, cls_conf in zip(class_names, confidences):
                        label_results.append(f"{cls_name}")
                        encode_mask_msg.labels.append(cls_name)
                        encode_mask_msg.confs.append(cls_conf)  

                if (len(label_results) > 0):
                # 处理掩码
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
                        except CvBridgeError as e:
                            rospy.logerr(f"Mask conversion error: {e}")
                        except Exception as e:
                            rospy.logerr(f"Mask processing error: {e}")
                        
                    if self.use_clip:
                        try:
                            text = self.clip_tockenizer(label_results).to("cuda:0")
                            with torch.no_grad():
                                text_features = self.clip_model.encode_text(text)
                                text_features /= text_features.norm(dim=-1, keepdim=True)
                                text_features = text_features.cpu().numpy()

                                # 计算余弦相似度并以矩阵形式输出
                                # cosine_sim = np.dot(text_features, text_features.T)
                                # print(cosine_sim)
                            

                                for i in range(0, text_features.shape[0]):
                                    # word_vector_msg.word_vector是 ros消息 float64[512], 将text_features转为float64[512]
                                    word_vector_msg = WordVector()
                                    word_vector_msg.word_vector = text_features[i].astype(np.float64)
                                    encode_mask_msg.word_vectors.append(word_vector_msg)
                        except Exception as e:
                            predict_success = False
                            rospy.logerr(f"CLIP error: {e}")
                        
                    # 发布结果消息
                    if predict_success and len(label_results) != 0:
                        self.result_pub.publish(encode_mask_msg)

            # 处理可视化和发布
            img = results[0].plot(
                boxes=True, masks=True, conf=True, labels=True
            )
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
            self.inference_count += 1

            t3 = rospy.Time.now()

            # 可视化
            if self.visualize:
                # 这里可以添加深度图和里程计信息的可视化
                cv2.imshow("YOLOE Detection", img)
                cv2.waitKey(1)
            
            # 打印处理时间
            if self.debug:
                rospy.logwarn(f"Predict time: {((t2 - t1).to_sec() * 1e3):.2f}ms")
                rospy.logwarn(f"Pub time: {((t3 - t2).to_sec() * 1e3):.2f}ms \n")


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
