#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import PromptMsg
from rospkg import RosPack
import os
from openai import OpenAI

# --- ROS 参数定义 ---
# 将推理结果发布到这个 Topic
RESULT_TOPIC = '/scene_graph/llm_ans'
PROMPT_TOPIC = '/scene_graph/prompt'
NODE_NAME    = 'LLM_API_NODE'

# --- 大模型 API 参数 ---
MODEL_TYPE = "qwen-flash"            
BASE_URL      = "https://dashscope.aliyuncs.com/compatible-mode/v1" # API 的 Base URL
API_KEY       = "sk-92320fd71e07458bb898abadca5c02eb"     # API_KEY
SYSTEM_PROMPT = "You are a great AI agent"

SYSTEM_PROMPT_AREA_PREDICT = "You are a great AI agent. Please predict some areas'type."
SYSTEM_PROMPT_AREA_CHOOSE = "You are a great AI agent. Please choose an area to explore."

# --- 全局变量定义 ---
client = None
result_publisher = None
area_predict_prompt_def_path = None
area_choosen_prompt_def_path = None

def initialize_llm_client():
    """
    初始化大模型 API 客户端。
    """
    global client
    try:
        client = OpenAI(
            base_url=BASE_URL,
            api_key=API_KEY,
        )
        rospy.loginfo("Initialize LLM client success.")
        return True
    except Exception as e:
        rospy.logfatal("Initialize LLM client failed: {}".format(e))
        return False


def call_llm_api(prompt_in: PromptMsg)->PromptMsg:
    """
    接收一个字符串 prompt, 调用您提供的火山方舟大模型 API, 并返回结果字符串。
    """

    # 确保客户端已成功初始化
    if client is None:
        error_msg = "Error in call_llm_api(): client is None."
        rospy.logerr(error_msg)
        return error_msg

    try:
        # 使用您提供的非流式（standard request）方式进行调用
        if (prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_ROOM_PREDICTION):
            rospy.logwarn("   * Room Prediction Mode !")
            completion = client.chat.completions.create(
                model=MODEL_TYPE,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT_AREA_PREDICT},
                    {"role": "user", "content": prompt_in.prompt}
                ],
                extra_body={"enable_thinking": False}
            )
        elif (prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_AREA_CHOOSE):
            rospy.logwarn("   * Area Choose Mode !")
            completion = client.chat.completions.create(
                model=MODEL_TYPE,
                 messages=[
                    {"role": "system", "content": SYSTEM_PROMPT_AREA_CHOOSE},
                    {"role": "user", "content": prompt_in.prompt}
                ],
                extra_body={"enable_thinking": False}
            )
        else:
            rospy.logerr("Error in call_llm_api(): prompt_type is not defined.")

        # 提取返回的文本内容
        llm_ans = PromptMsg()
        llm_ans.header.stamp = rospy.Time.now()
        llm_ans.answer       = completion.choices[0].message.content
        llm_ans.prompt_id    = prompt_in.prompt_id
        llm_ans.option       = PromptMsg.SEND_ANSWER
        return llm_ans

    except Exception as e:
        error_message = "Error in call_llm_api(): {}".format(e)
        rospy.logerr(error_message)
        return error_message


def prompt_callback(message: PromptMsg):
    rospy.logwarn(" ** Recved Prompt id : {}".format(message.prompt_id))
    rospy.logwarn(" **             Type : {}".format(message.prompt_type))
    rospy.loginfo(" **            Prompt: \n{}".format(message.prompt))
    time1 = rospy.Time.now()
    inference_result = call_llm_api(prompt_in=message)
    time2 = rospy.Time.now()
    rospy.logwarn(" ** LLM Inference Time: {}s".format((time2 - time1).to_sec()))
    rospy.logwarn(" ** LLM Answer: \n*=> {}".format(inference_result.answer))
    if result_publisher is not None:
        result_publisher.publish(inference_result)
        rospy.loginfo("Result Published ! \n")

def main():
    """
    主函数，用于初始化 ROS 节点、订阅者和发布者。
    """
    global result_publisher
    global area_predict_prompt_def_path
    global area_choosen_prompt_def_path

    global SYSTEM_PROMPT_AREA_PREDICT
    global SYSTEM_PROMPT_AREA_CHOOSE

    rospy.init_node(NODE_NAME, anonymous=True)
    if not initialize_llm_client():
        return  # 如果初始化失败，则退出节点
    
    area_predict_prompt_def_path = os.path.join(RosPack().get_path('scene_graph'), 'prompts_definition', 'room_prediction_syspt.txt')
    area_choosen_prompt_def_path = os.path.join(RosPack().get_path('scene_graph'), 'prompts_definition', 'area_choose_syspt.txt')

    with open(area_predict_prompt_def_path, 'r') as f:
        SYSTEM_PROMPT_AREA_PREDICT = f.read()
    rospy.loginfo("System Prompt: {}\n\n".format(SYSTEM_PROMPT_AREA_PREDICT))

    with open(area_choosen_prompt_def_path, 'r') as f:
        SYSTEM_PROMPT_AREA_CHOOSE = f.read()
    rospy.loginfo("System Prompt: {}\n\n".format(SYSTEM_PROMPT_AREA_CHOOSE))

    result_publisher = rospy.Publisher(RESULT_TOPIC, PromptMsg, queue_size=10)
    rospy.Subscriber(PROMPT_TOPIC, PromptMsg, prompt_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass