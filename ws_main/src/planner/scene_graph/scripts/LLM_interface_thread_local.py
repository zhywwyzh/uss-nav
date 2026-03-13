#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scene_graph.msg import PromptMsg
from rospkg import RosPack
import os
from openai import OpenAI
import queue
import threading
import sys
import json

from loguru import logger

RESULT_TOPIC = '/scene_graph/llm_ans'
PROMPT_TOPIC = '/scene_graph/prompt'
NODE_NAME = 'LLM_API_NODE'

MODEL_TYPE = ""
BASE_URL = "http://127.0.0.1:2231/v1"
API_KEY = "EMPTY"

SYSTEM_PROMPT_AREA_PREDICT = None
SYSTEM_PROMPT_AREA_CHOOSE = None
SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE = None
SYSTEM_PROMPT_DF_DEMO = None

client = None
model_name = None
result_publisher = None
prompt_queue = queue.Queue(maxsize=100)


def clear_proxy_env():
    """清理代理环境，避免本地 OpenAI 兼容服务被代理劫持。"""
    for key in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY",
                "http_proxy", "https_proxy", "all_proxy"):
        os.environ.pop(key, None)


def initialize_llm_client():
    """初始化本地 OpenAI 兼容客户端"""
    global client, model_name
    try:
        clear_proxy_env()
        client = OpenAI(base_url=BASE_URL, api_key=API_KEY)
        model_name = MODEL_TYPE
        if not model_name:
            models = client.models.list().data
            if len(models) == 0:
                raise RuntimeError("No local OpenAI-compatible models are available.")
            model_name = models[0].id
        logger.success("✅ LLM client initialized successfully.")
        logger.info("📌 Local model selected: {}", model_name)
        return True
    except Exception as e:
        logger.critical("🔥 Failed to initialize LLM client: {}", e)
        return False


def call_llm_api(prompt_in: PromptMsg) -> PromptMsg:
    """调用本地 OpenAI 兼容大模型 API"""
    prompt_id = prompt_in.prompt_id

    if client is None:
        error_msg = f"❌ [ID: {prompt_id}] Error: LLM client is not initialized."
        logger.error(error_msg)
        err_ans = PromptMsg()
        err_ans.answer = error_msg
        err_ans.prompt_id = prompt_id
        return err_ans

    try:
        mode_str = ""
        messages = []
        if prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_ROOM_PREDICTION:
            mode_str = "Room Prediction"
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT_AREA_PREDICT},
                {"role": "user", "content": prompt_in.prompt}
            ]
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_EXPL_PREDICTION:
            mode_str = "Area Choose"
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT_AREA_CHOOSE},
                {"role": "user", "content": prompt_in.prompt}
            ]
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_TERMINATE_OBJ_ID:
            mode_str = "Terminate Object ID Choose"
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE},
                {"role": "user", "content": prompt_in.prompt}
            ]
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_DF_DEMO:
            mode_str = "DF Demo"
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT_DF_DEMO},
                {"role": "user", "content": prompt_in.prompt}
            ]
        else:
            raise ValueError(f"Invalid prompt_type: {prompt_in.prompt_type}")

        logger.info(f"   [ID: {prompt_id}] 🤖 Calling LLM in [{mode_str}] mode...")
        completion = client.chat.completions.create(
            model=model_name,
            messages=messages
        )

        llm_ans = PromptMsg()
        llm_ans.header.stamp = rospy.Time.now()
        llm_ans.answer = completion.choices[0].message.content
        llm_ans.prompt_id = prompt_id
        llm_ans.option = PromptMsg.SEND_ANSWER
        return llm_ans

    except Exception as e:
        error_message = f"❌ [ID: {prompt_id}] Error during LLM API call: {e}"
        logger.error(error_message)
        err_ans = PromptMsg()
        err_ans.answer = error_message
        err_ans.prompt_id = prompt_id
        return err_ans


def prompt_callback(message: PromptMsg):
    """生产者：将 ROS 消息放入队列，并附带追踪ID"""
    if not prompt_queue.full():
        logger.info(f"📥 [ID: {message.prompt_id}] Received message. Added to queue. (Queue size: {prompt_queue.qsize() + 1})")
        prompt_queue.put(message)
    else:
        logger.warning(f"⚠️ [ID: {message.prompt_id}] Queue is full! Discarding message.")


def llm_processing_worker():
    """消费者：从队列中处理任务，所有日志都带追踪ID"""
    logger.info("👷 Worker thread started. Ready for tasks.")
    while not rospy.is_shutdown():
        try:
            message = prompt_queue.get(timeout=1.0)
            prompt_id = message.prompt_id

            logger.warning(f"⚙️ [ID: {prompt_id}] Picked from queue. Starting processing...")

            time1 = rospy.Time.now()
            inference_result = call_llm_api(prompt_in=message)
            time2 = rospy.Time.now()

            duration = (time2 - time1).to_sec()
            logger.success(f"⏱️ [ID: {prompt_id}] LLM inference finished in {duration:.3f}s")

            log_answer_content = ""
            try:
                answer_obj = json.loads(inference_result.answer)
                log_answer_content = json.dumps(answer_obj, indent=2, ensure_ascii=False)
            except json.JSONDecodeError:
                log_answer_content = inference_result.answer

            log_prompt_content = ""
            try:
                if message.prompt_type == PromptMsg.PROMPT_TYPE_EXPL_PREDICTION:
                    prompt_obj = json.loads(message.prompt)
                    log_prompt_content = json.dumps(prompt_obj, indent=2, ensure_ascii=False)
                else:
                    log_prompt_content = message.prompt
            except json.JSONDecodeError:
                log_prompt_content = message.prompt

            logger.info(f"💡 [ID: {prompt_id}] Prompt Input: \n{log_prompt_content}")
            logger.info(f"💡 [ID: {prompt_id}] Received answer:\n{log_answer_content}")

            if result_publisher is not None:
                result_publisher.publish(inference_result)
                logger.success(f"📤 [ID: {prompt_id}] Result published successfully!")

            prompt_queue.task_done()

        except queue.Empty:
            continue
        except Exception as e:
            logger.error(f"🔥 [Worker Thread] An unexpected error occurred: {e}")


def main():
    logger.remove()
    logger.add(
        sys.stdout,
        colorize=True,
        format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> - <level>{message}</level>",
        level="INFO"
    )

    logger.info("🚀 Starting LLM API Node...")
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    global BASE_URL, API_KEY, MODEL_TYPE
    BASE_URL = rospy.get_param("~llm_base_url", BASE_URL)
    API_KEY = rospy.get_param("~llm_api_key", API_KEY)
    MODEL_TYPE = rospy.get_param("~llm_model_name", MODEL_TYPE)

    if not initialize_llm_client():
        return

    worker_thread = threading.Thread(target=llm_processing_worker, daemon=True)
    worker_thread.start()

    global SYSTEM_PROMPT_AREA_PREDICT, SYSTEM_PROMPT_AREA_CHOOSE, SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE, SYSTEM_PROMPT_DF_DEMO
    try:
        rospack = RosPack()
        pkg_path = rospack.get_path('scene_graph')
        area_predict_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'room_prediction_syspt.txt')
        area_choosen_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'area_choose_syspt.txt')
        terminate_obj_id_choose_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'terminate_id_choose_syspt.txt')
        df_demo_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'df_demo_syspt.txt')

        with open(area_predict_prompt_def_path, 'r') as f:
            SYSTEM_PROMPT_AREA_PREDICT = f.read()
        logger.info("📄 System Prompt for [Area Prediction] loaded.")

        with open(area_choosen_prompt_def_path, 'r') as f:
            SYSTEM_PROMPT_AREA_CHOOSE = f.read()
        logger.info("📄 System Prompt for [Area Choose] loaded.")

        with open(terminate_obj_id_choose_prompt_def_path, 'r') as f:
            SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE = f.read()
        logger.info("📄 System Prompt for [Terminate Object ID Choose] loaded.")

        with open(df_demo_prompt_def_path, 'r') as f:
            SYSTEM_PROMPT_DF_DEMO = f.read()
        logger.info("📄 System Prompt for [DF Demo] loaded.")

    except Exception as e:
        logger.error(f"❌ Failed to load system prompts: {e}")

    global result_publisher
    result_publisher = rospy.Publisher(RESULT_TOPIC, PromptMsg, queue_size=10)
    rospy.Subscriber(PROMPT_TOPIC, PromptMsg, prompt_callback)

    logger.success("✅ Node is fully running. Waiting for prompts on topic: {}", PROMPT_TOPIC)
    rospy.spin()
    logger.info("🛑 Node is shutting down.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
