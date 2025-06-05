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
from tracking.car_tracker import CarTracker, PIDController
from inference.infer import *

class ServoControlThread(threading.Thread):
    def __init__(self, uservo_controller, position_queue, debug=False):
        super().__init__()
        self.uservo_controller = uservo_controller
        self.position_queue = position_queue
        self.debug = debug
        self.stop_event = threading.Event()
        self.daemon = True  # 主线程退出，这个线程自动退出
        self.start()  # 线程创建后自动启动
        self.cruise = True
        self.uservo_controller.set_yaw(0)
        self.uservo_controller.set_pitch(0)
    def run(self):
        last_kf_state = None
        cycle_count = 0  # 记录累积周期数
        missed_count = 0
        
        while not self.stop_event.is_set():
            t1 = time.time()
            try:
                # 尝试获取最新状态
                kf_state = self.position_queue.get(timeout=0.01)
                last_kf_state = kf_state
                cycle_count = 1  # 新数据来了，重置计数器
            except queue.Empty:
                if last_kf_state is None:
                    time.sleep(0.005)
                    continue
                else:
                    missed_count += 1
                if missed_count > 60:
                    self.cruise = True
                else:
                    cycle_count += 1  # 累加周期数
            if cycle_count > 35:
                continue
            if not self.cruise:
                self.cruise = False
                dt = cycle_count * 0.005
                missed_count = 0
                # 用累积dt预测下一位置
                x, y = last_kf_state[0, 0], last_kf_state[1, 0]
                vx, vy = last_kf_state[2, 0], last_kf_state[3, 0]
                ax, ay = last_kf_state[4, 0], last_kf_state[5, 0]

                pred_x = x + vx * dt + 0.5 * ax * dt ** 2
                pred_y = y + vy * dt + 0.5 * ay * dt ** 2

                # 不用更新last_kf_state中的值，因为这是估算未来位置

                # 发指令给舵机
                yaw_delta = 15 / 320 * (pred_x - 320)
                pitch_delta = 15 / 320 * (pred_y - 240)
                print(f"##############{pred_x}#########3")
                print(f"----------------{pred_y}-----------")
                yaw_now = self.uservo_controller.get_yaw()
                pitch_now = self.uservo_controller.get_pitch()
                self.uservo_controller.set_yaw(yaw_delta + yaw_now)
                self.uservo_controller.set_pitch(pitch_delta + pitch_now)

                if self.debug:
                    print(f"[MOVE-THREAD] Cycle {cycle_count}, dt={dt:.3f}s, Predicted Pos: ({pred_x:.2f}, {pred_y:.2f})")

                # time.sleep(dt_base)
                # self.uservo_controller.cruise()
                time.sleep(0.005)
                t2 = time.time()
                print(f"++++++{t2-t1}")
            else:
                self.uservo_controller.cruise()
                time.sleep(0.005)



    def stop(self):
        self.stop_event.set()
        self.join()

class track_process(object):
    def __init__(self, model_path=NET_PATH, car_controller:RobotController=None, uservo_controller:UservoController=None, PID_controller:PIDController=None,
                 output_array=None, output_lock=None, input_shape=None, resized_shape=None):
        # 使用YoLov5TRT初始化模型
        self.model = YoLov5TRT(model_path)
        self.trackers = {}
        self.finding = 0
        self.position_queue = queue.Queue(maxsize=1)
        self.car_controller = car_controller
        self.uservo_controller = uservo_controller
        self.uservo_controller.set_yaw(0)
        self.uservo_controller.set_pitch(0)
        self.PID_controller:PIDController = PID_controller
        # 共享内存相关
        self.output_array = output_array
        self.output_lock = output_lock
        self.input_shape = input_shape
        self.resized_shape = resized_shape
        # 线程自动启动
        # self.servo_thread = ServoControlThread(self.uservo_controller, self.position_queue, debug=DEBUG)
    def push_kf_state(self,kf_state):
        if self.position_queue.full():
            _ = self.position_queue.get_nowait()  # 先丢弃旧数据，保证队列不阻塞
        self.position_queue.put_nowait(kf_state)

    def update_tracker(self,tracker, cx, cy, bbox, dt):
        tracker.car_point = (cx, cy) 
        tracker.car_box = bbox
        kf_state = tracker.update_position(dt)

        return kf_state
    
    def move(self,point):
        yaw_delta = 15 / 320 * ( - point[0] + 320 )
        pitch_delta = 15 / 320 * ( - point[1] + 240)
        yaw_now, pitch_now = self.uservo_controller.get_yaw(), self.uservo_controller.get_pitch()
        self.uservo_controller.set_yaw(yaw_delta+yaw_now)
        self.uservo_controller.set_pitch(pitch_delta+pitch_now)
        

    def track(self,input_lock):

        trackers = {}  # 保存每个 ID 的 CarTracker
        last_time = time.time()
        pid_controller = PIDController(K_x=PID_K_X, K_y=PID_K_Y, uservo=self.uservo_controller)
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            start_time = time.time()
            input_np = np.frombuffer(input_array, dtype=np.uint8).reshape(input_shape)
            with input_lock:
                frame = input_np.copy()  # 拷贝一份，避免修改共享内存数据
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
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    if class_id not in trackers:
                        trackers[class_id] = CarTracker((cx, cy), [x1, y1, x2, y2])

                    tracker = trackers[class_id]
                    kf_state = self.update_tracker(tracker,cx,cy,[x1,y1,x2,y2],dt)
                    cv2.circle(frame, tracker.pred_position, 5, (0, 0, 255), -1)
                    # self.push_kf_state(kf_state=kf_state)
                    output_x, output_y = pid_controller.update(tracker.pred_position)
                    pid_controller.move(output_x, output_y)  
                    
            if not is_detected and self.uservo_controller is not None:
                self.uservo_controller.cruise()
            cv2.imshow("Detection", frame)
            cv2.waitKey(1)
            end_time = time.time()
            print(f"\033[92mFPS:{1/(end_time - start_time):.2f}\033[0m")
# # ------------------------------- 
# # 主程序入口
# # -------------------------------

if __name__ == "__main__" and True:
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

        main_process.track(input_lock=input_lock)
    except KeyboardInterrupt:
        print("Interrupted, stopping...")
    finally:
        if cam is not None:
            cam.stop()
        if main_process.servo_thread is not None:
            # main_process.servo_thread.stop()
            cv2.destroyAllWindows()