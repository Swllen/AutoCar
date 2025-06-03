"""
An example that uses TensorRT's Python api to make inferences.
"""
import ctypes
import os
import shutil
import random
import sys
import threading
import time
import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
from config import *
from yolov5.yolov5_det_trt import *
from vision.camera_init import camera_init
import vision.camera_remote as camera_remote

PLUGIN_LIBRARY = "yolov5/build/libmyplugins.so"

ctypes.CDLL(PLUGIN_LIBRARY)

yolov5_wrapper = YoLov5TRT(NET_PATH)
class Final_result:
    def __init__(self):
        self.boxes = [0,0,0,0]
        self.conf = 0.0

def infer(frame):
    pred = []
    k = 0
    pred.append(Final_result())
    results, use_time  = yolov5_wrapper.infer(frame)
    print(f"Use time: {use_time:.2f} s")
    if results is None:
        print("No results found.")
    else:
        print(f"Results found: {len(results)} objects detected.")
        for result in results:
            if pred[k].conf < result['conf'][0]:
                pred[k].boxes = result['boxes'][0]
                pred[k].conf = result['conf'][0]
                print(pred[k].boxes)
                print(f"conf:{pred[k].conf}")
            
    return pred

if __name__ == "__main__":
    cap = camera_init()
    camera_remote.set_queue(REMOTE_IMAGE_QUEUE)

    flask_thread = threading.Thread(target=camera_remote.run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    while True:
        ret, frame = cap.read()
        print(f"Frame shape: {frame.shape}")
        if not ret:
            print("Error: Could not read frame.")
            continue

        # 处理图像（比如上下翻转）
        frame = cv2.flip(frame, 0)

        # 执行推理
        preds = infer([frame])

        for pred in preds:
            x1, y1, x2, y2 = map(int, pred.boxes)
            conf = pred.conf
            # 画矩形框
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 写置信度文本
            cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        REMOTE_IMAGE_QUEUE.put(frame)