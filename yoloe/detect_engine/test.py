#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO detection API 的 CLI 测试客户端（无 ROS 依赖）。

该脚本负责：
1. 读取本地图像并编码为 base64。
2. 发送给 detect_engine/api.py 的 /detect 接口。
3. 打印返回的检测结果（bbox/conf/cls/name 与耗时）。
4. 可选：在图上绘制检测框并保存可视化结果。
"""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path

import cv2
import requests


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO detect API test client")
    parser.add_argument("--image", required=True, help="输入图像路径")
    parser.add_argument("--api-url", default="http://127.0.0.1:2251", help="detect API 地址")
    parser.add_argument("--conf", type=float, default=None, help="覆盖服务端 conf（0~1）")
    parser.add_argument("--iou", type=float, default=None, help="覆盖服务端后 NMS iou（0~1）")
    parser.add_argument("--agnostic", action="store_true", help="覆盖服务端 agnostic_nms")
    parser.add_argument("--imgsz", type=int, default=None, help="覆盖服务端 imgsz")
    parser.add_argument("--stamp", type=float, default=None, help="回显时间戳")
    parser.add_argument("--save-viz", default="", help="可视化结果保存路径（可选）")
    parser.add_argument("--timeout", type=float, default=30.0, help="HTTP 超时秒数")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_bgr = cv2.imread(args.image)
    if image_bgr is None:
        raise FileNotFoundError(f"failed to read image: {args.image}")
    ok, buf = cv2.imencode(".jpg", image_bgr)
    if not ok:
        raise RuntimeError("failed to encode image as jpeg")

    payload: dict = {"image_base64": base64.b64encode(buf.tobytes()).decode("ascii")}
    if args.stamp is not None:
        payload["stamp"] = args.stamp
    if args.conf is not None:
        payload["conf"] = args.conf
    if args.iou is not None:
        payload["iou"] = args.iou
    if args.agnostic:
        payload["agnostic_nms"] = True
    if args.imgsz is not None:
        payload["imgsz"] = args.imgsz

    resp = requests.post(f"{args.api_url.rstrip('/')}/detect", json=payload, timeout=args.timeout)
    resp.raise_for_status()
    data = resp.json()
    print(json.dumps(data, indent=2, ensure_ascii=False))

    if args.save_viz:
        viz = image_bgr.copy()
        for det in data.get("detections", []):
            x1, y1, x2, y2 = [int(round(v)) for v in det["bbox"]]
            cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['name']} {det['conf']:.2f}"
            cv2.putText(viz, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        out = Path(args.save_viz)
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), viz)
        print(f"visualization saved to {out}")


if __name__ == "__main__":
    main()
