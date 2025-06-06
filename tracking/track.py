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
from inference.infer import *
# -------------------------------
# 卡尔曼滤波器类（用于平滑装甲板中心点）
# -------------------------------

# This KalmanFilter is desinged to track 2D point and it will introduces errors in camera movement
# To resolve this, we should track 3D point. 
# #TODO: How to get target's (x,y,z) points? 
class ArmorKalmanFilter:
    def __init__(self, dt=1.0):
        self.kf = KalmanFilter(dim_x=6, dim_z=2)

        # 状态转移矩阵 F（包含加速度项）
        self.kf.F = np.array([[1, 0, dt, 0, 0.5*dt**2, 0],
                              [0, 1, 0, dt, 0, 0.5*dt**2],
                              [0, 0, 1, 0, dt, 0],
                              [0, 0, 0, 1, 0, dt],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])

        # 观测矩阵 H（只观测位置）
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0]])
        # 初始协方差矩阵 P
        # self.kf.P = np.array([[1, 0, 0, 0, 0, 0],
        #                       [0, 1, 0, 0, 0, 0],
        #                       [0, 0, 1, 0, 0, 0],
        #                       [0, 0, 0, 1, 0, 0],
        #                       [0, 0, 0, 0, 1, 0],
        #                       [0, 0, 0, 0, 0, 1]])
        self.kf.P = np.diag([100, 100, 100, 100, 100, 100])


        # 观测噪声协方差矩阵 R
        self.kf.R = np.diag([2.0, 2.0])  # x方向噪声比y大

        # 过程噪声协方差矩阵 Q（可调）
        # self.kf.Q = np.array([[1, 0, 0, 0, 0, 0],
        #                       [0, 1, 0, 0, 0, 0],
        #                       [0, 0, 1, 0, 0, 0],
        #                       [0, 0, 0, 1, 0, 0],
        #                       [0, 0, 0, 0, 1, 0],
        #                       [0, 0, 0, 0, 0, 1]])
        self.kf.Q = np.diag([0.1, 0.1, 1.0, 1.0, 5.0, 5.0])

        self.kf.x = np.zeros((6,1))

        self.initialized = False

    def update(self, point):
        if point is None:
            return None
        
        cx = point[0]
        cy = point[1]
        z = np.array([cx, cy])

        if not self.initialized:
            self.kf.x[:2] = z.reshape((2, 1))  # 位置初始化
            self.kf.x[2:] = 0  # 速度和加速度置零
            self.initialized = True

        self.kf.predict()
        self.kf.update(z)
        return int(self.kf.x[0]), int(self.kf.x[1])

# -------------------------------
# 每辆车一个追踪器
# -------------------------------
class CarTracker:
    def __init__(self, car_point, car_box, dt=1.0):
        self.car_point = car_point #car_point if for prediction
        self.car_box = car_box #car_box is for visualization 
        self.pred_position = None
        self.missed_frames = 0
        self.history = deque(maxlen=5)
        self.missed_car_point = None
        self.missed_car_box = None
        self.missed_pred_point = None
        self.kf = ArmorKalmanFilter(dt)

    def update_position(self):
        # add the current point to history, when the target is lost, we can use the history to predict the position
        self.history.append(self.car_point)
        self.missed_frames = 0
        self.pred_position = self.kf.update(self.car_point)
        if DEBUG:
            logger.debug(f"[KF] pred_X: {self.pred_position[0]:.4f}, pred_Y: {self.pred_position[1]:.4f}")
        return self.pred_position
        
    def guess_position(self):
        # print(f'####{self.missed_car_point}####')
        self.history.append(self.missed_car_point)
        self.missed_pred_point = self.kf.update(self.missed_car_point)
    
    def loss_frame(self):
        print(f"目标丢失{self.missed_frames}帧")
        self.missed_frames += 1


    def polynomial_trend_predict(self, order=1):
        """
        使用多项式趋势拟合历史点，并预测当前位置。
        order: 多项式阶数，1为线性，2为抛物线
        """
        if len(self.history) < order + 1:
            self.missed_car_point = self.car_point
            return

        x = [i for i in range(len(self.history))]
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]

        coef_x = np.polyfit(x, xs, order)
        coef_y = np.polyfit(x, ys, order)

        next_idx = len(self.history)
        pred_x = int(np.polyval(coef_x, next_idx))
        pred_y = int(np.polyval(coef_y, next_idx))

        self.missed_car_point = (pred_x, pred_y)

    def calculate_missed_box(self):
        x_offset = (self.car_box[2]-self.car_box[0])/2
        y_offset = (self.car_box[3]-self.car_box[1])/2
        self.missed_car_box = [self.missed_car_point[0]- x_offset, self.missed_car_point[0] + x_offset, 
                               self.missed_car_point[1] - y_offset, self.missed_car_point[1] + y_offset]

# -------------------------------
# PID控制器类
# -------------------------------
class PIDController:
    def __init__(self, K_x, K_y, uservo:UservoSer, setpoint=(0.5, 0.5), frame_size=[640, 480], dead_zone=0.00):
        """
        K_x: PID参数列表 [kp_x, ki_x, kd_x]
        K_y: PID参数列表 [kp_y, ki_y, kd_y]
        setpoint: 目标值（画面中心）- normalized to [0.5, 0.5]
        """
        self.kp_x = K_x[0]
        self.ki_x = K_x[1]
        self.kd_x = K_x[2]
        self.kp_y = K_y[0]
        self.ki_y = K_y[1]
        self.kd_y = K_y[2]
        self.setpoint = setpoint  # 目标值（画面中心）
        self.frame_size = frame_size  # 视频帧大小
        # 初始化横向和纵向的PID参数
        self.previous_drror_x = 0
        self.integral_x = 0
        self.previous_drror_y = 0
        self.integral_y = 0
        self.max_yaw_angle = 15  # 最大偏航角
        self.max_pitch_angle = 15  # 最大俯仰角
        self.dead_zone = dead_zone  # 死区阈值
        self.uservo = uservo

    def update(self, measured_value = None):
        if measured_value is None:
           measured_value = [0, 0]  # 或 return 0, 0 更安全
        nomalized_measured_value = (
            measured_value[0] / self.frame_size[0],
            measured_value[1] / self.frame_size[1]
        )


        # 计算横向和纵向误差
        error_x = nomalized_measured_value[0] - self.setpoint[0]
        error_y = nomalized_measured_value[1] - self.setpoint[1]

        # 应用死区
        if abs(error_x) < self.dead_zone:
            error_x = 0
        if abs(error_y) < self.dead_zone:
            error_y = 0

        # PID 输出计算
        self.integral_x += error_x
        derivative_x = error_x - self.previous_drror_x
        output_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x

        self.integral_y += error_y
        derivative_y = error_y - self.previous_drror_y
        output_y = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y

        self.previous_drror_x = error_x
        self.previous_drror_y = error_y
        yaw_delta = output_x * self.max_yaw_angle
        pitch_delta = output_y * self.max_pitch_angle
        if DEBUG:
            logger.debug(f"[PID] messured_x: {nomalized_measured_value[0]:.4f}, messured_y: {nomalized_measured_value[1]:.4f}")
            logger.debug(f"[PID] Error X: {error_x:.4f}, Error Y: {error_y:.4f}")
            logger.debug(f"[PID] Output X: {output_x:.4f}, Output Y: {output_y:.4f}")

        return yaw_delta,pitch_delta

    def move(self, output_x, output_y):
        yaw_now, pitch_now = self.uservo.get_yaw(), self.uservo.get_pitch()
        yaw_delta = output_x * self.max_yaw_angle
        pitch_delta = output_y * self.max_pitch_angle
        self.uservo.set_yaw(yaw_now-yaw_delta)
        self.uservo.set_pitch(pitch_delta+pitch_now)
        if DEBUG:
            logger.debug(f"[MOVE] Pitch Now: {pitch_now:.3f}, Yaw Now: {yaw_now:.3f}")
            logger.debug(f"[MOVE] Pitch Delta: {pitch_delta:.4f}, Yaw Delta: {yaw_delta:.4f}")

        

# # -------------------------------
# # 示例偏航角/俯仰角计算（根据tvec）
# # -------------------------------
def compute_yaw_pitch(tvec):
    x, y, z = tvec.flatten()
    yaw = math.degrees(math.atan2(x, z))
    pitch = math.degrees(math.atan2(-y, math.sqrt(x**2 + z**2)))
    return yaw, pitch

class track_process(object):
    def __init__(self,model_path=NET_PATH,car_controller:RobotController=None, uservo_ser:UservoSer=None, PID_controller:PIDController=None):
        self.model = infer_progress(model_path)
        self.trackers = {}
        self.finding = 0
        self.car_controller:RobotController = car_controller
        self.uservo_ser:UservoSer = uservo_ser
        self.PID_controller:PIDController = PID_controller

    def track(self):
        loss_frame = 100
        cap = camera_init()
        trackers = {}  # 保存每个 ID 的 CarTracker
        pid_controller = PIDController(K_x=PID_K_X, K_y=PID_K_Y, uservo=self.uservo_ser)
        while True:
            start_time = time.time() 
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                continue
            results = self.model.model_infer([frame])
            is_detected = 0
            print(f"results:{len(results)}")
            if len(results) != 0:
                for result in results:
                    class_id = result.classid
                    x1, y1, x2, y2 = map(int, result.boxes)
                    cx = int((result.boxes[0] + result.boxes[2]) / 2)
                    cy = int((result.boxes[1] + result.boxes[3]) / 2)
                    is_detected = 1             
                    # # 绘制测量点 (目标框中心)
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    # Create a new tracker if it doesn't exist
                    if class_id not in trackers:
                        trackers[class_id] = CarTracker((cx, cy), [x1, y1, x2, y2])
                    
                    # 这里是卡尔曼滤波器
                    tracker = trackers[class_id]
                    tracker.car_point = (cx, cy) 
                    tracker.car_box = [x1, y1, x2, y2]
                    pred_position = tracker.update_position()
                    cv2.circle(frame, tracker.pred_position, 5, (0, 0, 255), -1)  # 表示预测点
                    # REMOTE_IMAGE_QUEUE.put(frame)
                    # PID 控制器调整云台
                    
                    yaw_delta, pitch_delta = pid_controller.update(pred_position)
                    self.uservo_ser.send_packet(yaw_delta,pitch_delta,is_detected)   
                loss_frame = 0
            cv2.imshow("Camera View", frame)
            cv2.waitKey(1)

            if not is_detected: 
                loss_frame += 1
                if loss_frame > 10:
                    self.uservo_ser.send_packet(0,0,is_detected)
                else:
                    continue

            end_time = time.time()
            print(f"\033[92mFPS:{1/(end_time - start_time)}\033[0m")
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()


                
            
            # # missed_frame Processing
            # if not is_detected and priority_id in trackers:
            #     print(f"\033[31m没有检测到目标,使用filter位置\033[0m")
            #     tracker = trackers['RS']
            #     tracker.loss_frame()
            #     tracker.polynomial_trend_predict() # 如果没有检测到目标，使用filter位置
            #     tracker.missed_pred_point = tracker.guess_position()
            #     tracker.calculate_missed_box()
            #     tracker.calculate_keypoints()
            #     # 绘制预测点 (卡尔曼滤波器预测的中心)
            #     cv2.circle(frame, tracker.missed_pred_point, 5, (0, 0, 255), -1)
            #     cv2.rectangle(frame, (int(tracker.missed_car_box[0]),int(tracker.missed_car_box[2])),
            #                     (int(tracker.missed_car_box[1]),int(tracker.missed_car_box[3])), (0, 255, 0), 2)
            #     # Solve PnP
            #     _, rvec, tvec = solve_pnp(points_dD, tracker.missed_keypoints , K_0)
            #     # 根据 PID 控制器的输出调节云台（这里只是显示输出结果，实际应用中会控制云台的硬件）
            #     output_x, output_y = pid_controller.update(tracker.pred_position)
            #     pid_controller.move(output_x, output_y)   
            #     print(f"\033[31m[PID INFO]\033[0mPID miseed_Control - X: {output_x}, Y: {output_y}")
            # # 如果该目标丢失超过最大帧数(10)，清除该追踪器
            # if priority_id in trackers and trackers[priority_id].missed_frames > 10:
            #     print(f"\033[33m目标 {priority_id} 丢失超过10帧，已移除追踪器。\033[0m")
            #     del trackers[priority_id]
# # ------------------------------- 
# # 主程序入口
# # -------------------------------

if __name__ == "__main__" and True:
    # try:
    # camera_remote.set_queue(REMOTE_IMAGE_QUEUE)
    # flask_thread = threading.Thread(target=camera_remote.run_flask)
    # flask_thread.daemon = True
    # flask_thread.start()
    # cam = CameraThread(0)
    # t = threading.Thread(target=pub_image, args=(cam,), daemon=True)
    # t.start()
    uservo = UservoSer(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)
    main_process = track_process(NET_PATH,uservo_ser=uservo)
    main_process.track()
    # except Exception as e:
    #     print(f"[ERROR]{e}")
if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    input_shape = (480, 640, 3)
    resized_shape = (3, 320, 320)

    input_array = Array(ctypes.c_uint8, int(np.prod(input_shape)), lock=False)
    output_array = Array(ctypes.c_float, int(np.prod(resized_shape)), lock=False)
    input_lock = Lock()
    output_lock = Lock()

    cam = CameraProcess(input_size=(640, 480), resize_size=(320, 320))
    model = YoLov5TRT(NET_PATH)

    # 启动摄像头采集进程
    cam.start(input_array, output_array, input_lock, output_lock)

    try:
        uservo = UservoSer(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)
        main_process = track_process(NET_PATH, uservo_ser=uservo)

        # 如果track()需要传参，请确保定义并传入对应参数
        main_process.track(output_array, output_lock, input_shape, resized_shape)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, exiting...")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # 终止摄像头采集进程
        cam.stop()
        # 如果有其他资源释放也放这里，比如关闭窗口
        import cv2
        cv2.destroyAllWindows()
