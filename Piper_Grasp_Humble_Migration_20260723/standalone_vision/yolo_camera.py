#!/usr/bin/env python3
"""纯 YOLO 摄像头实时识别（12 类）。

相比 gui_detect.py：去掉了 HSV 正方体检测，只走 YOLO，避免同一个方块被
YOLO 和 HSV 各画一个框；并用 class-agnostic NMS + 手动 IoU 合并，消除
「实物类 vs 纸面类」对同一物体重复框的问题。

用法：
  python yolo_camera.py --weights runs/detect/runs/train/full_12cls/weights/best.pt --realsense
  python yolo_camera.py --weights <模型.pt> --camera 4          # 普通摄像头
"""

import argparse
import sys
import time

import cv2
import numpy as np
from ultralytics import YOLO


CLASS_NAMES = {
    0: "green_bottle", 1: "orange_bottle", 2: "purple_bottle",
    3: "red_cube", 4: "yellow_cube", 5: "blue_cube",
    6: "paper_green_bottle", 7: "paper_orange_bottle", 8: "paper_purple_bottle",
    9: "paper_red_cube", 10: "paper_yellow_cube", 11: "paper_blue_cube",
}
CLASS_COLORS = {
    0: (0, 255, 0), 1: (0, 165, 255), 2: (255, 0, 255),
    3: (0, 0, 255), 4: (0, 255, 255), 5: (255, 0, 0),
    6: (0, 150, 0), 7: (0, 110, 190), 8: (160, 0, 160),
    9: (0, 0, 160), 10: (0, 190, 190), 11: (160, 0, 0),
}


def iou(a, b):
    """两个 (x1,y1,x2,y2) 框的交并比。"""
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def merge_duplicates(dets, iou_thr=0.5):
    """按置信度从高到低，把 IoU 超过阈值的框当作同一物体，只保留最高分那个。"""
    dets = sorted(dets, key=lambda d: d[1], reverse=True)
    kept = []
    for d in dets:
        if any(iou(d[2:6], k[2:6]) > iou_thr for k in kept):
            continue
        kept.append(d)
    return kept


def parse_args():
    parser = argparse.ArgumentParser(description="纯 YOLO 摄像头实时识别")
    parser.add_argument("--weights", default="runs/detect/runs/train/full_12cls/weights/best.pt", help="模型权重")
    parser.add_argument("--camera", type=int, default=4, help="普通摄像头编号")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--iou", type=float, default=0.6, help="NMS IoU 阈值")
    parser.add_argument("--merge-iou", type=float, default=0.5, help="重复框合并 IoU 阈值")
    parser.add_argument("--fps", type=float, default=10.0, help="目标帧率（每秒处理帧数上限，0=不限）")
    parser.add_argument("--device", default="auto", help="推理设备：auto/cpu/0（GPU），工控机无 GPU 用 cpu")
    parser.add_argument("--realsense", action="store_true", help="使用 Intel RealSense RGB 流")
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.weights)

    if args.realsense:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        for _ in range(30):
            pipe.wait_for_frames()
        print("RealSense RGB 流就绪 (640x480)")
    else:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"无法打开摄像头 /dev/video{args.camera}")
            sys.exit(1)

    conf = args.conf
    target_interval = 1.0 / args.fps if args.fps > 0 else 0.0
    win = "YOLO 12类 | +/- 置信度 | s 截图 | q 退出"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 640)

    print(f"运行中... 目标 {args.fps:.0f} fps | +/- 置信度 | s 截图 | q 退出")
    fps = 0.0
    t0 = time.time()

    while True:
        loop_start = time.time()
        if args.realsense:
            frames = pipe.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf:
                continue
            frame = np.asanyarray(cf.get_data())
        else:
            ret, frame = cap.read()
            if not ret:
                continue

        # class-agnostic NMS：跨类别也会互相抑制，避免实物/纸面重复框
        res = model.predict(frame, conf=conf, iou=args.iou, agnostic_nms=True, verbose=False, device=args.device)

        dets = []
        if res and res[0].boxes is not None:
            boxes = res[0].boxes
            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                c = float(boxes.conf[i])
                cid = int(boxes.cls[i])
                dets.append((cid, c, int(x1), int(y1), int(x2), int(y2)))

        dets = merge_duplicates(dets, args.merge_iou)

        # 绘制
        display = frame.copy()
        for cid, c, x1, y1, x2, y2 in dets:
            color = CLASS_COLORS.get(cid, (0, 255, 255))
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            label = f"{CLASS_NAMES[cid]} {c:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(display, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(display, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        now = time.time()
        dt = now - t0
        t0 = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 / dt
        cv2.putText(display, f"FPS:{fps:.1f} conf:{conf:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("+"), ord("=")):
            conf = min(0.95, conf + 0.05)
            print(f"conf = {conf:.2f}")
        elif key in (ord("-"), ord("_")):
            conf = max(0.05, conf - 0.05)
            print(f"conf = {conf:.2f}")
        elif key == ord("s"):
            name = f"screenshot_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(name, display)
            print(f"已保存截图: {name}")

        # ── 帧率限制 ──
        if target_interval > 0:
            remain = target_interval - (time.time() - loop_start)
            if remain > 0:
                time.sleep(remain)

    if args.realsense:
        pipe.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
