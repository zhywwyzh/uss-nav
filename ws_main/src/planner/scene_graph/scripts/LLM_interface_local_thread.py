#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""SceneGraph 与 VLA-Swarm 共用的本地 OpenAI 兼容推理节点。"""

import base64
import json
import os
import queue
import sys
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from loguru import logger
from openai import OpenAI
from rospkg import RosPack
from scene_graph.msg import PromptMsg, VLASwarmObservation
from sensor_msgs.msg import Image

from vla_swarm_prompt_router import (
    create_answer,
    load_prompt_specs,
    resolve_prompt_request,
    structured_error,
)


PROMPT_TOPIC = "/scene_graph/prompt"
RESULT_TOPIC = "/scene_graph/llm_ans"
SMALL_MAP_TOPIC = "/vla_swarm/small_map"
OBSERVATION_TOPIC = "/vla_swarm/observation"
NODE_NAME = "LLM_LOCAL_API_NODE"

# 普通 SceneGraph 与 VLA-Swarm 文本 Prompt 默认使用 8002（Qwen3.6-35B-A3B）。
TEXT_BASE_URL = os.environ.get(
    "SCENE_GRAPH_LOCAL_TEXT_BASE_URL",
    "http://119.45.181.200:8002/v1",
)
# SmallMap 和前视 Observation 等视觉 Prompt 默认使用 8002。
VISION_BASE_URL = os.environ.get(
    "SCENE_GRAPH_LOCAL_VISION_BASE_URL",
    "http://119.45.181.200:8002/v1",
)
API_KEY = os.environ.get("SCENE_GRAPH_LOCAL_API_KEY", "EMPTY")
TEXT_MODEL = os.environ.get("SCENE_GRAPH_LOCAL_TEXT_MODEL", "")
VISION_MODEL = os.environ.get("SCENE_GRAPH_LOCAL_VISION_MODEL", "")

REQUEST_TIMEOUT = 120.0
TEMPERATURE = 0.2
SMALL_MAP_MAX_AGE = 5.0
OBSERVATION_MAX_AGE = 30.0

text_client = None
vision_client = None
text_model_name = ""
vision_model_name = ""
result_publisher = None
prompt_routes = {}
prompt_queue = queue.Queue(maxsize=100)

image_bridge = CvBridge()
small_map_lock = threading.Lock()
latest_small_map = None
latest_small_map_receive_time = rospy.Time()
observation_lock = threading.Lock()
observation_cache = {}


def _create_client(base_url):
    """创建本地 OpenAI 兼容客户端。"""
    return OpenAI(
        api_key=API_KEY,
        base_url=str(base_url).strip(),
        timeout=REQUEST_TIMEOUT,
    )


def _resolve_model_name(client, configured_model, endpoint_name):
    """优先使用显式模型名，否则从本地服务发现第一个可用模型。"""
    model_name = str(configured_model or "").strip()
    if model_name:
        return model_name

    models = client.models.list().data
    if not models:
        raise RuntimeError("{} does not expose any model".format(endpoint_name))
    return str(models[0].id)


def initialize_llm_clients():
    """初始化 2233 文本客户端和 2235 视觉客户端。"""
    global text_client, vision_client, text_model_name, vision_model_name

    try:
        text_client = _create_client(TEXT_BASE_URL)
        vision_client = _create_client(VISION_BASE_URL)
        text_model_name = _resolve_model_name(
            text_client, TEXT_MODEL, TEXT_BASE_URL
        )
        vision_model_name = _resolve_model_name(
            vision_client, VISION_MODEL, VISION_BASE_URL
        )
    except Exception as error:
        logger.error("Failed to initialize local LLM clients: {}", error)
        return False

    logger.info(
        "Text endpoint: {} model={}", TEXT_BASE_URL, text_model_name
    )
    logger.info(
        "Vision endpoint: {} model={}", VISION_BASE_URL, vision_model_name
    )
    return True


def small_map_callback(message):
    """把最新 SmallMap 编码为 JPEG，供视觉工作线程使用。"""
    global latest_small_map, latest_small_map_receive_time

    try:
        cv_image = image_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        encoded, jpeg_data = cv2.imencode(".jpg", cv_image)
    except Exception as error:
        logger.error("Failed to convert SmallMap image: {}", error)
        return

    if not encoded:
        logger.error("Failed to encode SmallMap image.")
        return

    with small_map_lock:
        latest_small_map = base64.b64encode(jpeg_data.tobytes()).decode("ascii")
        latest_small_map_receive_time = rospy.Time.now()


def observation_callback(message):
    """按任务会话、观察批次和方向缓存 FSM 固化的前视图像。"""
    cache_key = (
        int(message.task_session_id),
        int(message.observation_batch_id),
        int(message.observation_index),
    )
    image_format = str(message.image.format or "jpeg").lower()
    mime_type = "image/png" if "png" in image_format else "image/jpeg"
    receive_time = rospy.Time.now()
    image_data = base64.b64encode(bytes(message.image.data)).decode("ascii")

    with observation_lock:
        observation_cache[cache_key] = {
            "data": image_data,
            "mime_type": mime_type,
            "receive_time": receive_time,
        }
        stale_keys = [
            key
            for key, value in observation_cache.items()
            if (receive_time - value["receive_time"]).to_sec()
            > OBSERVATION_MAX_AGE
        ]
        for key in stale_keys:
            observation_cache.pop(key, None)


def _prompt_observation_context(prompt_text):
    """从 VLA-Swarm Prompt JSON 中读取当前 session 和 batch。"""
    try:
        prompt_data = json.loads(prompt_text)
        return (
            int(prompt_data["task_session_id"]),
            int(prompt_data["observation_batch_id"]),
        ), None
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        return None, structured_error(
            "invalid_observation_context",
            "Prompt lacks task_session_id/observation_batch_id: {}".format(
                error
            ),
        )


def _observation_indexes(visual_input):
    """把路由中的视觉输入名称转换为 Observation 序号。"""
    if visual_input == "observation_batch":
        return [0, 1, 2, 3], None
    if visual_input.startswith("observation_"):
        try:
            index = int(visual_input.rsplit("_", 1)[1])
        except ValueError:
            index = -1
        if 0 <= index <= 3:
            return [index], None
    return None, structured_error(
        "invalid_observation_route",
        "Unsupported visual input: {}".format(visual_input),
    )


def _observation_visual_contents(visual_input, prompt_text):
    """获取当前 Prompt 所需的 Observation 图像内容。"""
    context, context_error = _prompt_observation_context(prompt_text)
    if context_error:
        return None, context_error

    indexes, index_error = _observation_indexes(visual_input)
    if index_error:
        return None, index_error

    current_time = rospy.Time.now()
    images = []
    missing_indexes = []
    with observation_lock:
        for observation_index in indexes:
            cached = observation_cache.get(
                (context[0], context[1], observation_index)
            )
            if (
                cached is None
                or (current_time - cached["receive_time"]).to_sec()
                > OBSERVATION_MAX_AGE
            ):
                missing_indexes.append(observation_index)
                continue
            images.append((observation_index, dict(cached)))

    if missing_indexes:
        return None, structured_error(
            "observation_not_ready",
            "Missing observations {} for session={} batch={}".format(
                missing_indexes, context[0], context[1]
            ),
        )

    visual_contents = []
    for observation_index, image in images:
        visual_contents.append(
            {
                "type": "text",
                "text": "Observation {}".format(observation_index),
            }
        )
        visual_contents.append(
            {
                "type": "image_url",
                "image_url": {
                    "url": "data:{};base64,{}".format(
                        image["mime_type"], image["data"]
                    )
                },
            }
        )
    return visual_contents, None


def _small_map_visual_contents():
    """读取新鲜的 SmallMap 图像内容。"""
    with small_map_lock:
        image_data = latest_small_map
        receive_time = latest_small_map_receive_time

    if (
        image_data is None
        or (rospy.Time.now() - receive_time).to_sec() > SMALL_MAP_MAX_AGE
    ):
        return None, structured_error(
            "small_map_not_ready",
            "No fresh SmallMap image is available on {}".format(
                SMALL_MAP_TOPIC
            ),
        )

    return [
        {
            "type": "image_url",
            "image_url": {
                "url": "data:image/jpeg;base64,{}".format(image_data)
            },
        }
    ], None


def create_request(route, prompt_text):
    """根据 Prompt 路由构造文本或多模态 OpenAI 请求。"""
    visual_input = route["visual_input"]
    if visual_input is None:
        return (
            text_client,
            text_model_name,
            [
                {"role": "system", "content": route["system_prompt"]},
                {"role": "user", "content": prompt_text},
            ],
            None,
        )

    if visual_input == "small_map":
        visual_contents, visual_error = _small_map_visual_contents()
    else:
        visual_contents, visual_error = _observation_visual_contents(
            visual_input, prompt_text
        )
    if visual_error:
        return None, None, None, visual_error

    messages = [
        {"role": "system", "content": route["system_prompt"]},
        {
            "role": "user",
            "content": [{"type": "text", "text": prompt_text}]
            + visual_contents,
        },
    ]
    return vision_client, vision_model_name, messages, None


def call_llm_api(prompt_in):
    """按 Prompt 类型调用 2233 文本端点或 2235 视觉端点。"""
    route, route_error = resolve_prompt_request(prompt_in, prompt_routes)
    if route_error:
        return create_answer(
            PromptMsg, prompt_in, route_error, rospy.Time.now()
        )

    client, model_name, messages, request_error = create_request(
        route, prompt_in.prompt
    )
    if request_error:
        return create_answer(
            PromptMsg, prompt_in, request_error, rospy.Time.now()
        )

    try:
        logger.info(
            "[ID: {}] Calling {} route={} model={}",
            prompt_in.prompt_id,
            "vision" if route["visual_input"] else "text",
            route["mode"],
            model_name,
        )
        completion = client.chat.completions.create(
            model=model_name,
            messages=messages,
            temperature=TEMPERATURE,
            extra_body={
                "chat_template_kwargs": {"enable_thinking": False},
            },
        )
        message = completion.choices[0].message
        answer_text = getattr(message, "content", None)
        if not answer_text:
            answer_text = getattr(message, "reasoning", None)
        if not answer_text:
            raise RuntimeError("Local model returned an empty answer")
        return create_answer(
            PromptMsg,
            prompt_in,
            str(answer_text).strip(),
            rospy.Time.now(),
        )
    except Exception as error:
        logger.error(
            "[ID: {}] Local LLM request failed: {}",
            prompt_in.prompt_id,
            error,
        )
        return create_answer(
            PromptMsg,
            prompt_in,
            structured_error("llm_request_failed", str(error)),
            rospy.Time.now(),
        )


def prompt_callback(message):
    """仅接收 Prompt 请求，答案消息不会重新进入处理队列。"""
    if message.option != PromptMsg.SEND_PROMPT:
        return
    if prompt_queue.full():
        logger.warning("[ID: {}] Prompt queue is full.", message.prompt_id)
        result_publisher.publish(
            create_answer(
                PromptMsg,
                message,
                structured_error(
                    "prompt_queue_full", "Local LLM request queue is full"
                ),
                rospy.Time.now(),
            )
        )
        return
    prompt_queue.put(message)


def llm_processing_worker():
    """按到达顺序处理 Prompt，避免本地模型并发争用显存。"""
    while not rospy.is_shutdown():
        try:
            message = prompt_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        try:
            result = call_llm_api(message)
            result_publisher.publish(result)
            logger.info("[ID: {}] Answer published.", message.prompt_id)
        except Exception as error:
            logger.error(
                "[ID: {}] Unexpected worker error: {}",
                message.prompt_id,
                error,
            )
        finally:
            prompt_queue.task_done()


def main():
    """初始化本地双端点 LLM 节点。"""
    logger.remove()
    logger.add(
        sys.stdout,
        colorize=True,
        format=(
            "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
            "<level>{level: <8}</level> | <level>{message}</level>"
        ),
        level="INFO",
    )
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    global TEXT_BASE_URL, VISION_BASE_URL, API_KEY
    global TEXT_MODEL, VISION_MODEL, REQUEST_TIMEOUT, TEMPERATURE
    global PROMPT_TOPIC, RESULT_TOPIC
    global SMALL_MAP_TOPIC, SMALL_MAP_MAX_AGE
    global OBSERVATION_TOPIC, OBSERVATION_MAX_AGE
    global prompt_routes, result_publisher

    TEXT_BASE_URL = rospy.get_param("~text_base_url", TEXT_BASE_URL)
    VISION_BASE_URL = rospy.get_param("~vision_base_url", VISION_BASE_URL)
    API_KEY = rospy.get_param("~api_key", API_KEY)
    TEXT_MODEL = rospy.get_param("~text_model", TEXT_MODEL)
    VISION_MODEL = rospy.get_param("~vision_model", VISION_MODEL)
    REQUEST_TIMEOUT = float(
        rospy.get_param("~request_timeout", REQUEST_TIMEOUT)
    )
    TEMPERATURE = float(rospy.get_param("~temperature", TEMPERATURE))
    PROMPT_TOPIC = rospy.get_param("~prompt_topic", PROMPT_TOPIC)
    RESULT_TOPIC = rospy.get_param("~answer_topic", RESULT_TOPIC)
    SMALL_MAP_TOPIC = rospy.get_param(
        "~small_map_topic", SMALL_MAP_TOPIC
    )
    SMALL_MAP_MAX_AGE = float(
        rospy.get_param("~small_map_max_age", SMALL_MAP_MAX_AGE)
    )
    OBSERVATION_TOPIC = rospy.get_param(
        "~observation_topic", OBSERVATION_TOPIC
    )
    OBSERVATION_MAX_AGE = float(
        rospy.get_param("~observation_max_age", OBSERVATION_MAX_AGE)
    )

    try:
        package_path = RosPack().get_path("scene_graph")
        prompt_routes = load_prompt_specs(PromptMsg, package_path)
    except Exception as error:
        logger.error("Failed to load Prompt routes: {}", error)
        return

    if not initialize_llm_clients():
        return

    result_publisher = rospy.Publisher(
        RESULT_TOPIC, PromptMsg, queue_size=10
    )
    rospy.Subscriber(
        PROMPT_TOPIC, PromptMsg, prompt_callback, queue_size=100
    )
    rospy.Subscriber(
        SMALL_MAP_TOPIC, Image, small_map_callback, queue_size=1
    )
    rospy.Subscriber(
        OBSERVATION_TOPIC,
        VLASwarmObservation,
        observation_callback,
        queue_size=10,
        buff_size=2**24,
    )

    worker_thread = threading.Thread(
        target=llm_processing_worker, daemon=True
    )
    worker_thread.start()
    logger.info(
        "Local LLM node ready: text={} vision={}",
        TEXT_BASE_URL,
        VISION_BASE_URL,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
