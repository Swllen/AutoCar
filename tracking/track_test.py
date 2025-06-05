from inference.infer import infer_once, YoLov5TRT  # 确认infer_once和YoLov5TRT能正确导入
import cv2
import numpy as np
import math
import time
from filterpy.kalman import KalmanFilter
from collections import deque
import torch
from control.Control import RobotController, UservoController
from config import *
from yolov5.predictor import *
import camera.camera_remote as camera_remote
from camera.camera import CameraThread,pub_image,image_queue
from Logger.Logger import logger
from inference.infer import *
from tracking.car_tracker import CarTracker

class track_process(object):
    def __init__(self, model_path=NET_PATH, car_controller:RobotController=None, uservo_controller:UservoController=None, PID_controller=None,
                 output_array=None, output_lock=None, input_shape=None, resized_shape=None):
        # 使用YoLov5TRT初始化模型
        self.model = YoLov5TRT(model_path)
        self.trackers = {}
        self.finding = 0
        self.position_queue = queue.Queue(maxsize=1)
        self.car_controller = car_controller
        self.uservo_controller = uservo_controller

        # 共享内存相关
        self.output_array = output_array
        self.output_lock = output_lock
        self.input_shape = input_shape
        self.resized_shape = resized_shape

    def push_kf_state(self,kf_state):
        if self.position_queue.full():
            _ = self.position_queue.get_nowait()  # 先丢弃旧数据，保证队列不阻塞
        self.position_queue.put_nowait(kf_state)

    def update_tracker(self,tracker, cx, cy, bbox, dt):
        tracker.car_point = (cx, cy) 
        tracker.car_box = bbox
        kf_state = tracker.update_position(dt)
        return kf_state

    def track(self):
        trackers = {}  # 保存每个 ID 的 CarTracker
        last_time = time.time()

        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            start_time = time.time()

            # 调用infer_once从共享内存读取推理结果
            results = infer_once(self.model, self.output_array, self.output_lock, self.input_shape, self.resized_shape)

            is_detected = False
            print(f"results:{len(results)}")
            if len(results) != 0:
                for result in results:
                    class_id = result.classid
                    x1, y1, x2, y2 = map(int, result.boxes)
                    cx = int((result.boxes[0] + result.boxes[2]) / 2)
                    cy = int((result.boxes[1] + result.boxes[3]) / 2)
                    is_detected = True              
                    # cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    if class_id not in trackers:
                        trackers[class_id] = CarTracker((cx, cy), [x1, y1, x2, y2])

                    tracker = trackers[class_id]
                    kf_state = self.update_tracker(tracker,cx,cy,[x1,y1,x2,y2],dt)
                    # cv2.circle(frame, tracker.pred_position, 5, (0, 0, 255), -1)
                    self.push_kf_state(kf_state=kf_state)

            # if not is_detected and self.uservo_controller is not None:
                # self.uservo_controller.cruise()

            end_time = time.time()
            print(f"\033[92mFPS:{1/(end_time - start_time):.2f}\033[0m")

        cap.release()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    input_shape = (480, 640, 3)
    resized_shape = (3, 320, 320)

    input_array = Array(ctypes.c_uint8, int(np.prod(input_shape)), lock=False)
    output_array = Array(ctypes.c_float, int(np.prod(resized_shape)), lock=False)
    input_lock = Lock()
    output_lock = Lock()

    cam = CameraProcess(input_size=(640, 480), resize_size=(320, 320))
    cam.start(input_array, output_array, input_lock, output_lock)

    try:
        uservo = UservoController(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)

        main_process = track_process(model_path=NET_PATH,
                                     uservo_controller=uservo,
                                     output_array=output_array,
                                     output_lock=output_lock,
                                     input_shape=input_shape,
                                     resized_shape=resized_shape)

        main_process.track()
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
