from Logger.Logger import logger
import sys
from config import *
from multiprocessing import  Process
import cv2
import numpy as np
import os
from datetime import datetime
import time
import pytz
import threading

def camera_init():
        # 启动摄像头读取主循环
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, INPUT_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, INPUT_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()
    else:
        logger.info("Camera initialized successfully.")
    return cap
class record_thread(threading.Thread):
    def __init__(self,input_array,input_lock,input_shape = (480, 640, 3)):
        super().__init__()
        self.stop_event = threading.Event()
        self.input_array = input_array
        self.input_lock = input_lock
        self.input_shape = input_shape

    def run(self):
        frame_np = np.frombuffer(self.input_array,dtype=np.uint8).reshape(self.input_shape)
    # with input_lock:
    #     frame = frame_np.copy()
        save_dir = 'images/camera_record/'
        os.makedirs(save_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        filename = os.path.join(save_dir, f"{timestamp}.avi")
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (0, 255, 255)
        thickness = 2
        position = (10, 30)

        h, w = self.input_shape[:2]
        fps = 60
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(filename, fourcc, fps, (w, h))

        logger.info(f"Recording started:{filename}")

        while True:
            with self.input_lock:
                frame = frame_np.copy()
                # 获取北京时间字符串
            # 获取北京时间字符串
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # 写时间戳文字到图像上
            cv2.putText(frame, timestamp, position, font, font_scale, color, thickness, cv2.LINE_AA)

            writer.write(frame)
            time.sleep(1.0 / fps)
    def stop(self):
        self.stop_event.set()
        logger.info("record_thread is exited")





class CameraProcess():
    def __init__(self, input_size=(640, 480), resize_size=(320, 320)):
        self.input_w, self.input_h = input_size
        self.resize_w, self.resize_h = resize_size

    def captureAndpreprocess(self, input_array,output_array,input_lock,output_lock):
        frame_np = np.frombuffer(input_array, dtype=np.uint8).reshape(self.input_h, self.input_w, 3)
        cap = camera_init()

        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            # frame = cv2.flip(frame, 0)
            with input_lock:
                np.copyto(frame_np, frame)
                input_np = np.frombuffer(input_array, dtype=np.uint8).reshape(self.input_h, self.input_w, 3)

            #Preprocess
        
            output_np = np.frombuffer(output_array, dtype=np.float32).reshape(3, self.resize_h, self.resize_w)
            
            image_raw = input_np.copy()
            h, w, _ = image_raw.shape
            image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)

            r_w = self.resize_w / w
            r_h = self.resize_h / h

            if r_h > r_w:
                tw, th = self.resize_w, int(r_w * h)
                tx1 = tx2 = 0
                ty1 = int((self.resize_h - th) / 2)
                ty2 = self.resize_h - th - ty1
            else:
                tw, th = int(r_h * w), self.resize_h
                tx1 = int((self.resize_w - tw) / 2)
                tx2 = self.resize_w - tw - tx1
                ty1 = ty2 = 0

            image = cv2.resize(image, (tw, th))
            image = cv2.copyMakeBorder(image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, value=(128, 128, 128))
            image = image.astype(np.float32) / 255.0
            image = np.transpose(image, [2, 0, 1])
            image = np.ascontiguousarray(image)
            with output_lock:
                np.copyto(output_np, image)
    def start(self, input_array, output_array, input_lock, output_lock):
        self.process = Process(target=self.captureAndpreprocess, args=(input_array, output_array, input_lock, output_lock))
        self.process.daemon = True  # 可选，随主进程退出
        self.process.start()

    def stop(self):
        if self.process is not None:
            self.process.terminate()
            self.process.join()

def visualize(input_array, input_lock, results, input_shape):
    # 取原始图像
    input_np = np.frombuffer(input_array, dtype=np.uint8).reshape(input_shape)
    with input_lock:
        frame = input_np.copy()  # 拷贝一份，避免修改共享内存数据

    for pred in results:
        x1, y1, x2, y2 = map(int, pred.boxes)
        conf = pred.conf
        # 画矩形框
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 写置信度文本
        cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Detection", frame)
    cv2.waitKey(1)

if __name__ == "__main__":
        # 初始化图像队列
    import time
    cap = camera_init()
    while True:
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            continue

        # 处理图像（比如上下翻转）
        frame = cv2.flip(frame, 0)

        cv2.imshow("frame",frame)
        k = cv2.waitKey(1)
        end_time = time.time()
        print(f"FPS:{1/(end_time - start_time):.4f}")
        if k == 27:         # 按下esc时，退出
            sys.exit()