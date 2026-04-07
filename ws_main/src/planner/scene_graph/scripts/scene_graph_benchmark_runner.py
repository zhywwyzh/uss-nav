#!/usr/bin/env python3

import json
import math
import os
import re
import time
from collections import Counter

import rospy
from nav_msgs.msg import Odometry
from openai import OpenAI
from scene_graph.msg import PromptMsg
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import tiktoken
except ImportError:
    tiktoken = None




BENCHMARK_PROMPT_TOPIC = "/scene_graph/benchmark_prompt"
BENCHMARK_ANSWER_TOPIC = "/scene_graph/benchmark_llm_ans"
BENCHMARK_METRICS_TOPIC = "/scene_graph/benchmark_metrics"

DEFAULT_BASE_URL = "http://127.0.0.1:2231/v1"
DEFAULT_API_KEY = "EMPTY"

ROOM_LABEL_NS = "scene_graph_room_label_text"
ROOM_EDGE_NS = "scene_graph_room_room_edge"
OBJ_ROOM_EDGE_NS = "scene_graph_obj_level"
OBJ_LABEL_NS = "obj_label"

TASK_TO_PROMPT_TYPE = {
    "room_prediction": PromptMsg.PROMPT_TYPE_ROOM_PREDICTION,
    "expl_prediction": PromptMsg.PROMPT_TYPE_EXPL_PREDICTION,
    "terminate_obj_id": PromptMsg.PROMPT_TYPE_TERMINATE_OBJ_ID,
    "df_demo": PromptMsg.PROMPT_TYPE_DF_DEMO,
}


def point_to_xyz(point):
    return (float(point.x), float(point.y), float(point.z))


def distance_2d(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def nearest_room_id(position, room_centers, max_dist=float("inf")):
    best_id = None
    best_dist = float("inf")
    for room_id, center in room_centers.items():
        d = distance_2d(position, center)
        if d < best_dist:
            best_dist = d
            best_id = room_id
    if best_dist > max_dist:
        return None
    return best_id


def parse_room_label(text):
    match = re.match(r"Area\[(\d+)\](.*)", text.strip())
    if match is None:
        return None, text.strip()
    room_id = int(match.group(1))
    label = match.group(2).strip() or "Unknown"
    return room_id, label


def parse_object_label(text, fallback_id):
    match = re.match(r"\[(\d+)\]\s+(.*)", text.strip())
    if match is None:
        return fallback_id, text.strip()
    return int(match.group(1)), match.group(2).strip()


def clear_proxy_env():
    for key in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"):
        os.environ.pop(key, None)


class MarkerSnapshotCollector(object):
    def __init__(self):
        self.scene_graph_msg = None
        self.object_all_vis_msg = None
        self.odom_msg = None

    def scene_graph_callback(self, msg):
        self.scene_graph_msg = msg

    def object_all_vis_callback(self, msg):
        self.object_all_vis_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def ready(self, require_objects):
        if self.scene_graph_msg is None:
            return False
        if require_objects and self.object_all_vis_msg is None:
            return False
        return True

    def build_snapshot(self, current_area_override=None):
        scene_graph = self._parse_scene_graph()
        objects = self._parse_objects(scene_graph["room_centers"], scene_graph["obj_room_pairs"])

        for obj in objects:
            room_id = obj.get("room_id")
            if room_id is not None and room_id in scene_graph["rooms"]:
                scene_graph["rooms"][room_id]["objects"].append(obj)

        current_area_id = current_area_override
        current_area_source = "override"
        if current_area_id is None:
            current_area_source = "inferred_from_odom"
            current_area_id = self._infer_current_area(scene_graph["room_centers"])

        for room in scene_graph["rooms"].values():
            room["objects"].sort(key=lambda item: item["id"])
            room["object_labels"] = [item["label"] for item in room["objects"]]
            room["object_count"] = len(room["objects"])

        room_list = []
        for room_id in sorted(scene_graph["rooms"]):
            room = scene_graph["rooms"][room_id]
            room_list.append(
                {
                    "id": str(room["id"]),
                    "label": room["label"],
                    "center": [
                        round(room["center"][0], 3),
                        round(room["center"][1], 3),
                        round(room["center"][2], 3),
                    ],
                    "neighbor_ids": [str(value) for value in sorted(room["neighbor_ids"])],
                    "object_count": room["object_count"],
                    "object_labels": room["object_labels"],
                    "objects": room["objects"],
                }
            )

        stamp = None
        if self.scene_graph_msg is not None and self.scene_graph_msg.markers:
            stamp = self.scene_graph_msg.markers[0].header.stamp.to_sec()

        return {
            "captured_at": time.time(),
            "scene_graph_stamp": stamp,
            "current_area_id": None if current_area_id is None else str(current_area_id),
            "current_area_source": current_area_source,
            "room_count": len(room_list),
            "object_count": len(objects),
            "rooms": room_list,
        }

    def _infer_current_area(self, room_centers):
        if self.odom_msg is None or not room_centers:
            return None
        pos = (
            float(self.odom_msg.pose.pose.position.x),
            float(self.odom_msg.pose.pose.position.y),
            float(self.odom_msg.pose.pose.position.z),
        )
        return nearest_room_id(pos, room_centers)

    def _parse_scene_graph(self):
        room_centers = {}
        room_labels = {}
        adjacency = {}
        obj_room_pairs = []

        for marker in self.scene_graph_msg.markers:
            if marker.ns == ROOM_LABEL_NS and marker.type == Marker.TEXT_VIEW_FACING:
                room_id, label = parse_room_label(marker.text)
                if room_id is None:
                    continue
                room_centers[room_id] = (
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                    float(marker.pose.position.z - 0.5),
                )
                room_labels[room_id] = label
                adjacency.setdefault(room_id, set())

        for marker in self.scene_graph_msg.markers:
            if marker.ns == ROOM_EDGE_NS and marker.type == Marker.LINE_LIST:
                points = list(marker.points)
                for idx in range(0, len(points), 2):
                    if idx + 1 >= len(points):
                        break
                    p1 = point_to_xyz(points[idx])
                    p2 = point_to_xyz(points[idx + 1])
                    room_a = nearest_room_id(p1, room_centers, max_dist=1.0)
                    room_b = nearest_room_id(p2, room_centers, max_dist=1.0)
                    if room_a is None or room_b is None or room_a == room_b:
                        continue
                    adjacency.setdefault(room_a, set()).add(room_b)
                    adjacency.setdefault(room_b, set()).add(room_a)
            elif marker.ns == OBJ_ROOM_EDGE_NS and marker.type == Marker.LINE_LIST:
                points = list(marker.points)
                for idx in range(0, len(points), 2):
                    if idx + 1 >= len(points):
                        break
                    obj_pos = point_to_xyz(points[idx])
                    room_pos = point_to_xyz(points[idx + 1])
                    room_id = nearest_room_id(room_pos, room_centers, max_dist=1.0)
                    if room_id is None:
                        continue
                    obj_room_pairs.append((obj_pos, room_id))

        rooms = {}
        for room_id, center in room_centers.items():
            rooms[room_id] = {
                "id": room_id,
                "label": room_labels.get(room_id, "Unknown"),
                "center": center,
                "neighbor_ids": adjacency.get(room_id, set()),
                "objects": [],
            }

        return {
            "rooms": rooms,
            "room_centers": room_centers,
            "obj_room_pairs": obj_room_pairs,
        }

    def _parse_objects(self, room_centers, obj_room_pairs):
        if self.object_all_vis_msg is None:
            return []

        objects = []
        for marker in self.object_all_vis_msg.markers:
            if marker.ns != OBJ_LABEL_NS or marker.type != Marker.TEXT_VIEW_FACING:
                continue

            object_id, label = parse_object_label(marker.text, marker.id)
            pos = (
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z - 0.1),
            )

            room_id = None
            best_dist = float("inf")
            for obj_pos, candidate_room_id in obj_room_pairs:
                d = distance_2d(pos, obj_pos)
                if d < best_dist:
                    best_dist = d
                    room_id = candidate_room_id
            if best_dist > 0.6:
                room_id = nearest_room_id(pos, room_centers, max_dist=5.0)

            objects.append(
                {
                    "id": int(object_id),
                    "label": label,
                    "pos": [round(pos[0], 3), round(pos[1], 3), round(pos[2], 3)],
                    "room_id": room_id,
                }
            )
        return sorted(objects, key=lambda item: item["id"])


class PromptSynthesizer(object):
    def __init__(self, target_text):
        self.target_text = target_text

    def build_payload(self, task_type, snapshot):
        if task_type == "expl_prediction":
            return self._build_expl_payload(snapshot)
        if task_type == "room_prediction":
            return self._build_room_payload(snapshot)
        if task_type == "terminate_obj_id":
            return self._build_object_selection_payload(snapshot, current_area_only=True)
        if task_type == "df_demo":
            return self._build_object_selection_payload(snapshot, current_area_only=False)
        raise ValueError("Unsupported task_type: {}".format(task_type))

    def _build_expl_payload(self, snapshot):
        areas = []
        for room in snapshot["rooms"]:
            object_counter = Counter(room["object_labels"])
            areas.append(
                {
                    "id": room["id"],
                    "label": room["label"],
                    "neighborAreas": room["neighbor_ids"],
                    "objectCount": room["object_count"],
                    "objectSummary": [
                        "{} x{}".format(label, count)
                        for label, count in sorted(object_counter.items())
                    ],
                }
            )
        return {
            "snapshotSource": "scene_graph_vis+object_all_vis",
            "task": "expl_prediction",
            "currentAreaId": snapshot["current_area_id"],
            "target": self.target_text,
            "areas": areas,
        }

    def _build_room_payload(self, snapshot):
        areas = []
        for room in snapshot["rooms"]:
            areas.append(
                {
                    "id": room["id"],
                    "currentLabel": room["label"],
                    "objects": [
                        {
                            "id": str(obj["id"]),
                            "label": obj["label"],
                            "pos": [str(obj["pos"][0]), str(obj["pos"][1])],
                        }
                        for obj in room["objects"]
                    ],
                }
            )
        return {
            "snapshotSource": "scene_graph_vis+object_all_vis",
            "task": "room_prediction",
            "areas": areas,
        }

    def _build_object_selection_payload(self, snapshot, current_area_only):
        objects = []
        for room in snapshot["rooms"]:
            if current_area_only and snapshot["current_area_id"] is not None:
                if room["id"] != snapshot["current_area_id"]:
                    continue
            for obj in room["objects"]:
                objects.append(
                    {
                        "id": str(obj["id"]),
                        "label": obj["label"],
                        "pos": [str(obj["pos"][0]), str(obj["pos"][1])],
                        "roomId": room["id"],
                        "roomLabel": room["label"],
                    }
                )
        return {
            "snapshotSource": "scene_graph_vis+object_all_vis",
            "task": "terminate_obj_id" if current_area_only else "df_demo",
            "currentAreaId": snapshot["current_area_id"],
            "target": self.target_text,
            "objects": objects,
        }


class BenchmarkLlmClient(object):
    def __init__(self, base_url, api_key, model_name, stream, use_usage_if_available):
        self.client = OpenAI(base_url=base_url, api_key=api_key)
        self.model_name = model_name
        self.stream = stream
        self.use_usage_if_available = use_usage_if_available

    def run(self, messages):
        start = time.perf_counter()
        usage = None
        output_text = ""
        ttft_ms = None

        if self.stream:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                stream=True,
            )
            chunks = []
            for chunk in response:
                now = time.perf_counter()
                if getattr(chunk, "usage", None) is not None:
                    usage = chunk.usage
                if not getattr(chunk, "choices", None):
                    continue
                choice = chunk.choices[0]
                delta = getattr(choice, "delta", None)
                piece = getattr(delta, "content", None) if delta is not None else None
                if piece:
                    if ttft_ms is None:
                        ttft_ms = (now - start) * 1000.0
                    chunks.append(piece)
            output_text = "".join(chunks)
        else:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
            )
            usage = getattr(response, "usage", None)
            output_text = response.choices[0].message.content

        end = time.perf_counter()
        latency_ms = (end - start) * 1000.0

        prompt_tokens = None
        completion_tokens = None
        total_tokens = None
        token_source = "estimate"

        if usage is not None and self.use_usage_if_available:
            prompt_tokens = getattr(usage, "prompt_tokens", None)
            completion_tokens = getattr(usage, "completion_tokens", None)
            total_tokens = getattr(usage, "total_tokens", None)
            if prompt_tokens is not None and completion_tokens is not None:
                token_source = "api_usage"

        if prompt_tokens is None:
            prompt_tokens = estimate_message_tokens(messages, self.model_name)
        if completion_tokens is None:
            completion_tokens = estimate_text_tokens(output_text, self.model_name)
        if total_tokens is None:
            total_tokens = prompt_tokens + completion_tokens

        return {
            "output_text": output_text,
            "ttft_ms": ttft_ms,
            "latency_ms": latency_ms,
            "prompt_tokens": prompt_tokens,
            "completion_tokens": completion_tokens,
            "total_tokens": total_tokens,
            "token_source": token_source,
        }


def estimate_text_tokens(text, model_name):
    if not text:
        return 0
    if tiktoken is not None:
        try:
            encoding = tiktoken.encoding_for_model(model_name)
        except Exception:
            encoding = tiktoken.get_encoding("cl100k_base")
        return len(encoding.encode(text))
    return len(text.split())


def estimate_message_tokens(messages, model_name):
    return sum(estimate_text_tokens(message.get("content", ""), model_name) for message in messages)


def load_system_prompt(task_type, default_pkg_path):
    filename_map = {
        "expl_prediction": "benchmark_expl_prediction_syspt.txt",
        "room_prediction": "benchmark_room_prediction_syspt.txt",
        "terminate_obj_id": "benchmark_terminate_obj_id_syspt.txt",
        "df_demo": "benchmark_df_demo_syspt.txt",
    }
    filepath = os.path.join(default_pkg_path, "prompts_definition", filename_map[task_type])
    with open(filepath, "r", encoding="utf-8") as file_obj:
        return file_obj.read()


def append_jsonl(output_path, record):
    if not output_path:
        return
    directory = os.path.dirname(output_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(output_path, "a", encoding="utf-8") as file_obj:
        file_obj.write(json.dumps(record, ensure_ascii=False) + "\n")


def wait_for_snapshot(collector, timeout_sec, require_objects):
    deadline = time.time() + timeout_sec
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if collector.ready(require_objects):
            return True
        if time.time() >= deadline:
            return False
        rate.sleep()
    return False


def main():
    rospy.init_node("scene_graph_benchmark_runner", anonymous=True)
    clear_proxy_env()

    task_type = rospy.get_param("~task_type", "expl_prediction").strip()
    if task_type not in TASK_TO_PROMPT_TYPE:
        raise ValueError("Unsupported task_type: {}".format(task_type))

    target_text = rospy.get_param("~target_text", "Find the most relevant area")
    model_name = rospy.get_param("~llm_model_name", "")
    base_url = rospy.get_param("~llm_base_url", DEFAULT_BASE_URL)
    api_key = rospy.get_param("~llm_api_key", DEFAULT_API_KEY)
    stream = rospy.get_param("~stream", True)
    use_usage_if_available = rospy.get_param("~use_usage_if_available", True)
    wait_timeout_sec = float(rospy.get_param("~wait_timeout_sec", 30.0))
    output_path = rospy.get_param(
        "~output_path",
        os.path.join(os.path.expanduser("~"), "scene_graph_benchmark_results.jsonl"),
    )
    scene_graph_topic = rospy.get_param("~scene_graph_topic", "/scene_graph/vis")
    object_all_vis_topic = rospy.get_param("~object_all_vis_topic", "/object_all_vis")
    odom_topic = rospy.get_param("~odom_topic", "/odom_world")
    current_area_id_param = rospy.get_param("~current_area_id", "")
    publish_prompt = rospy.get_param("~publish_prompt", True)
    publish_answer = rospy.get_param("~publish_answer", True)

    current_area_override = None
    if current_area_id_param not in ("", None):
        current_area_override = int(current_area_id_param)

    collector = MarkerSnapshotCollector()
    rospy.Subscriber(scene_graph_topic, MarkerArray, collector.scene_graph_callback, queue_size=2)
    rospy.Subscriber(object_all_vis_topic, MarkerArray, collector.object_all_vis_callback, queue_size=2)
    rospy.Subscriber(odom_topic, Odometry, collector.odom_callback, queue_size=10)

    prompt_pub = rospy.Publisher(BENCHMARK_PROMPT_TOPIC, PromptMsg, queue_size=2)
    answer_pub = rospy.Publisher(BENCHMARK_ANSWER_TOPIC, PromptMsg, queue_size=2)
    metrics_pub = rospy.Publisher(BENCHMARK_METRICS_TOPIC, String, queue_size=2)

    require_objects = task_type in ("room_prediction", "terminate_obj_id", "df_demo")
    rospy.loginfo("Waiting for benchmark snapshot on %s%s", scene_graph_topic,
                  " and " + object_all_vis_topic if require_objects else "")
    if not wait_for_snapshot(collector, wait_timeout_sec, require_objects):
        raise RuntimeError("Timed out waiting for benchmark input topics.")

    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    system_prompt = load_system_prompt(task_type, pkg_path)
    snapshot = collector.build_snapshot(current_area_override=current_area_override)
    if snapshot["room_count"] == 0:
        raise RuntimeError("Snapshot does not contain any rooms. /scene_graph/vis is not ready for benchmarking.")
    synthesizer = PromptSynthesizer(target_text=target_text)
    payload = synthesizer.build_payload(task_type, snapshot)
    user_prompt = json.dumps(payload, ensure_ascii=False, indent=2)
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt},
    ]

    if not model_name:
        client = OpenAI(base_url=base_url, api_key=api_key)
        model_name = client.models.list().data[0].id

    prompt_id = int(time.time() * 1000) % 2147483647
    if publish_prompt:
        prompt_msg = PromptMsg()
        prompt_msg.header.stamp = rospy.Time.now()
        prompt_msg.prompt_id = prompt_id
        prompt_msg.option = PromptMsg.SEND_PROMPT
        prompt_msg.prompt_type = TASK_TO_PROMPT_TYPE[task_type]
        prompt_msg.prompt = user_prompt
        prompt_pub.publish(prompt_msg)

    runner = BenchmarkLlmClient(
        base_url=base_url,
        api_key=api_key,
        model_name=model_name,
        stream=stream,
        use_usage_if_available=use_usage_if_available,
    )
    result = runner.run(messages)

    if publish_answer:
        answer_msg = PromptMsg()
        answer_msg.header.stamp = rospy.Time.now()
        answer_msg.prompt_id = prompt_id
        answer_msg.option = PromptMsg.SEND_ANSWER
        answer_msg.prompt_type = TASK_TO_PROMPT_TYPE[task_type]
        answer_msg.answer = result["output_text"]
        answer_pub.publish(answer_msg)

    metrics_record = {
        "prompt_id": prompt_id,
        "task_type": task_type,
        "model_name": model_name,
        "stream": stream,
        "target_text": target_text,
        "snapshot": snapshot,
        "prompt_payload": payload,
        "prompt_text": user_prompt,
        "system_prompt": system_prompt,
        "response_text": result["output_text"],
        "prompt_tokens": result["prompt_tokens"],
        "completion_tokens": result["completion_tokens"],
        "total_tokens": result["total_tokens"],
        "token_source": result["token_source"],
        "ttft_ms": result["ttft_ms"],
        "latency_ms": result["latency_ms"],
        "output_path": output_path,
    }
    append_jsonl(output_path, metrics_record)
    metrics_pub.publish(String(data=json.dumps(metrics_record, ensure_ascii=False)))

    rospy.loginfo("Benchmark finished. task=%s prompt_tokens=%s completion_tokens=%s ttft_ms=%s latency_ms=%.3f",
                  task_type,
                  metrics_record["prompt_tokens"],
                  metrics_record["completion_tokens"],
                  "null" if metrics_record["ttft_ms"] is None else "{:.3f}".format(metrics_record["ttft_ms"]),
                  metrics_record["latency_ms"])


if __name__ == "__main__":
    main()
