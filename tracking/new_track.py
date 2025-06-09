import cv2
import numpy as np
import math
import time
from filterpy.kalman import KalmanFilter
from collections import deque
import torch
from control.Control import RobotController, UservoSer
from config import *
from yolov5.predictor import *
import camera.camera_remote as camera_remote
from camera.camera import CameraThread,pub_image,image_queue
from Logger.Logger import logger
from tracking.car_tracker import CarTracker, update_yaw_pitch
from inference.infer import *



class ServoControlThread(threading.Thread):
    def __init__(self, uservo_ser:UservoSer, position_queue,is_detected,debug=False):
        super().__init__()
        self.uservo_ser = uservo_ser
        self.position_queue = position_queue
        self.debug = debug
        self.stop_event = threading.Event()
        self.daemon = True
        self.last_kf_state = None
        self.cycle_count = 0
        self.is_detected_lock = threading.Lock() 
        is_detected = is_detected
        self.start()
        
        
    def predict_position(self, state, dt):
        x, y = state[0, 0], state[1, 0]
        vx, vy = state[2, 0], state[3, 0]
        ax, ay = state[4, 0], state[5, 0]

        pred_x = x + vx * dt + 0.5 * ax * dt ** 2
        pred_y = y + vy * dt + 0.5 * ay * dt ** 2
        return pred_x, pred_y


    def run(self):
        global is_detected
        while not self.stop_event.is_set():
            with self.is_detected_lock:
                if not is_detected:
                    print("No detection, sending zero packet.")
                    self.uservo_ser.send_packet(0,0,0)
                    self.cycle_count = 0
            try:
                kf_state = self.position_queue.get(timeout=0.01)
                self.last_kf_state = kf_state
            except queue.Empty:
                if self.last_kf_state is None:
                    continue

            with self.is_detected_lock:
                if is_detected:
                    self.cycle_count += 1
                    print(self.last_kf_state)
                    dt = self.cycle_count * 0.025  # 预测时间间隔
                    pred_x, pred_y = self.predict_position(self.last_kf_state, dt)
                    yaw_delta , pitch_delta = update_yaw_pitch(pred_x, pred_y)
                    self.uservo_ser.send_packet(yaw_delta,pitch_delta,is_detected)
                    print(f"Predicted position: yaw={yaw_delta}, pitch={pitch_delta}")

            time.sleep(0.03)

    def stop(self):
        self.stop_event.set()
        self.join()

class track_process(object):
    def __init__(self, model_path=NET_PATH, car_controller:RobotController=None, uservo_ser:UservoSer=None,
                 input_array=None,output_array=None, output_lock=None, input_shape=None, resized_shape=None,input_lock=None):
        # 使用YoLov5TRT初始化模型
        self.model = YoLov5TRT(model_path)
        self.trackers = {}
        self.finding = 0
        self.position_queue = queue.Queue(maxsize=1)
        self.car_controller = car_controller
        self.uservo_ser = uservo_ser
        # is_detected = 0  # 作为实例变量
        # 共享内存相关
        self.output_array = output_array
        self.output_lock = output_lock
        self.input_lock = input_lock
        self.input_shape = input_shape
        self.resized_shape = resized_shape
        self.input_array = input_array
        # 线程自动启动
        # self.servo_thread = ServoControlThread(self.uservo_ser, self.position_queue, is_detected, debug=DEBUG)
        self.is_detected_lock = threading.Lock() 
        
        # WIN_SIGNAL
        self.center_range = 50  # 画面中心的像素范围
        self.victory_time = 2.0  # 胜利条件：持续2秒
        self.target_timings = {}  # 每个目标的计时信息
        self.frame_shape = (640, 480)  # 帧大小
        self.enable_victory_check = True  # Flag to control victory checking
        self.lock = threading.Lock()  # Lock for thread safety
        self.pred_position = None
        
    def push_kf_state(self,kf_state):
        if self.position_queue.full():
            _ = self.position_queue.get_nowait()  # 先丢弃旧数据，保证队列不阻塞
        self.position_queue.put_nowait(kf_state)

    def update_tracker(self,tracker, cx, cy, bbox, dt):
        tracker.car_point = (cx, cy) 
        tracker.car_box = bbox
        kf_state = tracker.update_position(dt)

        return kf_state
    def is_in_center(self, point):
        if point is None:
            return False
        else:
            """检查点是否在画面中心范围内"""
            frame_center = (self.frame_shape[0] // 2, self.frame_shape[1] // 2)
            distance = math.hypot(point[0] - frame_center[0], point[1] - frame_center[1])
            return distance <= self.center_range

    def win_signal(self):
                # 胜利判断及丢帧处理
        class_id = 'paper'
        if class_id not in self.target_timings:
            self.target_timings[class_id] = {'start_time': None, 'last_position': None, 'missed_frames': 0,
                                            'was_in_center': False}
        timing = self.target_timings[class_id]
        with self.lock:
            if self.enable_victory_check and self.is_in_center(self.pred_position):
                print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
                if not timing['was_in_center']:
                    timing['start_time'] = time.time()
                timing['was_in_center'] = True
                if timing['start_time'] is not None and time.time() - timing['start_time'] >= self.victory_time:
                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                    victory_thread = threading.Thread(target=self.car_controller.victory())
                    victory_thread.start()
                    # self.car_controller.beep_on()
            else:
                timing['was_in_center'] = False
                timing['start_time'] = None
        # 处理丢帧
        if timing['missed_frames'] > 0 and timing['was_in_center']:
            if timing['last_position'] and np.linalg.norm(
                    np.array(self.pred_position) - np.array(timing['last_position'])) > 50:
                timing['start_time'] = None
                timing['was_in_center'] = False
        timing['last_position'] = self.pred_position
        timing['missed_frames'] = 0
    

    def track(self):
        global is_detected
        trackers = {}  # 保存每个 ID 的 CarTracker
        last_time = time.time()
        loss_frame = 0
        while True:
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            start_time = time.time()
            input_np = np.frombuffer(self.input_array, dtype=np.uint8).reshape(self.input_shape)
            with self.input_lock:
                frame = input_np.copy()  # 拷贝一份，避免修改共享内存数据
            # 调用infer_once从共享内存读取推理结果
            results = infer_once(self.model, self.output_array, self.output_lock, self.input_shape, self.resized_shape)
            if len(results) != 0:
                print("update update")
                for result in results:
                    with self.is_detected_lock:
                        is_detected = 1
                    loss_frame = 0
                    class_id = result.classid
                    x1, y1, x2, y2 = map(int, result.boxes)
                    cx = int((result.boxes[0] + result.boxes[2]) / 2)
                    cy = int((result.boxes[1] + result.boxes[3]) / 2)             
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    if class_id not in trackers:
                        trackers[class_id] = CarTracker((cx, cy), [x1, y1, x2, y2])

                    tracker = trackers[class_id]
                    kf_state = self.update_tracker(tracker,cx,cy,[x1,y1,x2,y2],dt)
                    cv2.circle(frame, tracker.pred_position, 5, (0, 0, 255), -1)
                    self.pred_position = tracker.pred_position
                    
                    self.push_kf_state(kf_state=kf_state)
                    pred_x = kf_state[0,0]
                    pred_y = kf_state[1,0]
                    vx = kf_state[2,0]
                    vy = kf_state[3,0]
                    pred_x = pred_x + vx*0.035
                    pred_y = pred_y + vy*0.035
                    yaw_delta,pitch_delta=update_yaw_pitch(pred_x,pred_y)
                    yaw_delta, pitch_delta = round(yaw_delta, 1), round(pitch_delta, 1)

                    self.uservo_ser.send_packet(yaw_delta,pitch_delta,is_detected)
                    self.win_signal()
                    
            else:
                loss_frame += 1
                if loss_frame > 20:
                    with self.is_detected_lock:
                        is_detected = 0
                    self.uservo_ser.send_packet(0,0,0)
            # cv2.imshow("Detection", frame)
            # cv2.waitKey(1)
            end_time = time.time()
            print(f"\033[92mFPS:{1/(end_time - start_time):.2f}\033[0m")
# # ------------------------------- 
# # 主程序入口
# # -------------------------------

if __name__ == "__main__" and True:
    global is_detected  # 声明 is_detected 为全局变量
    is_detected = 0  # 初始化为 0，表示没有检测到目标
    input_shape = (480, 640, 3)
    resized_shape = (3, 320, 320)

    input_array = Array(ctypes.c_uint8, int(np.prod(input_shape)), lock=False)
    output_array = Array(ctypes.c_float, int(np.prod(resized_shape)), lock=False)
    input_lock = Lock()
    output_lock = Lock()

    cam = CameraProcess(input_size=(640, 480), resize_size=(320, 320))
    cam.start(input_array, output_array, input_lock, output_lock)
    record = record_thread(input_array,input_lock,input_shape)
    record.start()
    try:
        uservo = UservoSer(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)

        main_process = track_process(model_path=NET_PATH,
                                     uservo_ser=uservo,
                                     output_array=output_array,
                                     output_lock=output_lock,
                                     input_array=input_array,
                                     input_lock=input_lock,
                                     input_shape=input_shape,
                                     resized_shape=resized_shape)

        main_process.track()
    except KeyboardInterrupt:
        print("Interrupted, stopping...")
    finally:
        if cam is not None:
            cam.stop()
        # if main_process.servo_thread is not None:
        #     # main_process.servo_thread.stop()
        #     cv2.destroyAllWindows()