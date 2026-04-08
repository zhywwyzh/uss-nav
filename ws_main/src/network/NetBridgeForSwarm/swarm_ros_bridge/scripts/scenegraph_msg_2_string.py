#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from scene_graph.msg import PromptMsg


PROMPT_TOPIC = '/scene_graph/prompt'
OUTPUT_TOPIC = '/scene_graph/json_text'
NODE_NAME = 'scenegraph_to_copaw'


class SceneGraphToCopawBridge(object):
    def __init__(self):
        output_topic = rospy.get_param('~output_topic', OUTPUT_TOPIC)
        self.publisher = rospy.Publisher(output_topic, String, queue_size=10)
        self.subscriber = rospy.Subscriber(PROMPT_TOPIC, PromptMsg, self.prompt_callback, queue_size=10)
        rospy.loginfo('Bridge ready: %s -> %s', PROMPT_TOPIC, output_topic)

    def prompt_callback(self, msg):
        prompt_msg = String()
        prompt_msg.data = msg.prompt
        self.publisher.publish(prompt_msg)
        rospy.loginfo('Forwarded prompt_id=%s to std_msgs/String topic.', msg.prompt_id)


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    SceneGraphToCopawBridge()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
