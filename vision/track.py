import cv2
import numpy as np
import math
import time
from ultralytics import YOLO
from filterpy.kalman import KalmanFilter
from collections import deque

# -------------------------------
# 卡尔曼滤波器类（用于平滑装甲板中心点）
# -------------------------------
class ArmorKalmanFilter:
    def __init__(self, dt=1.0, process_noise=5e-1, measurement_noise=2.0):
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
        self.kf.P *= 1000.

        # 观测噪声协方差矩阵 R
        self.kf.R *= measurement_noise

        # 过程噪声协方差矩阵 Q（可调）
        self.kf.Q = np.eye(6) * process_noise

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
    def __init__(self, car_id, car_point, car_box, dt=1.0, process_noise=1e-2, measurement_noise=1.0):
        self.car_id = car_id
        self.keypoints = None
        self.car_point = car_point
        self.car_box = car_box
        self.new_position = None
        self.missed_frames = 0
        self.history = deque(maxlen=5)
        self.missed_car_point = None
        self.missed_car_box = None
        self.missed_pred_point = None
        self.missed_keypoints = [[[0,0],[0,0],[0,0],[0,0]]]
        self.kf = ArmorKalmanFilter(dt, process_noise, measurement_noise)

    def update_position(self):
        self.history.append(self.car_point)
        self.missed_frames = 0
        return self.kf.update(self.car_point)
    def guess_position(self):
        print(f'####{self.missed_car_point}####')
        self.history.append(self.missed_car_point)
        return self.kf.update(self.missed_car_point)
    def loss_frame(self):
        print(f"目标丢失{self.missed_frames}zhen，ID: {self.car_id}")
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

    def calculate_box(self):
        x_offset = (self.car_box[1]-self.car_box[0])/2
        y_offset = (self.car_box[3]-self.car_box[2])/2
        self.missed_car_box = [self.missed_car_point[0]- x_offset, self.missed_car_point[0] + x_offset, 
                               self.missed_car_point[1] - y_offset, self.missed_car_point[1] + y_offset]
    def calculate_keypoints(self):
        x_offset = self.missed_car_point[0] - self.car_point[0]
        y_offset = self.missed_car_point[1] - self.car_point[1]
        print(f"x_offset: {x_offset}, y_offset: {y_offset}")
        for i in range(len(self.keypoints)):
            # print(f"keypoints[i][0]: {self.keypoints[i][0]}")
            try:
                self.missed_keypoints[i][0] += self.keypoints[i][0] + x_offset
            except:
                print(f"missed_keypoints[i]: {self.missed_keypoints[i]}")
                self.missed_keypoints[i][0] = self.keypoints[i][0] + x_offset
                print(f"missed_keypoints[i][0]: {self.missed_keypoints[i][0]}")
            print(f"missed_keypoints[i][0]: {self.missed_keypoints[i][0]}")
            try:
                self.missed_keypoints[i][1] += self.keypoints[i][0] + y_offset
            except:
                print(f"missed_keypoints[i]: {self.missed_keypoints[i]}")
                self.missed_keypoints[i][1] = self.keypoints[i][0] + y_offset
                print(f"missed_keypoints[i][1]: {self.missed_keypoints[i][1]}")
        print(f"over calculate_keypoints")

# -------------------------------
# PID控制器类
# -------------------------------
class PIDController:
    def __init__(self, K_x, K_y, setpoint=(0.5, 0.5),frame_size=(1440, 1080),dead_zone=0.01):
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
        self.previous_error_x = 0
        self.integral_x = 0
        self.previous_error_y = 0
        self.integral_y = 0
        self.max_yaw_angle = 0.1  # 最大偏航角
        self.max_pitch_angle = 0.1  # 最大俯仰角
        self.dead_zone = dead_zone  # 死区阈值

    def update(self, measured_value):
        nomalized_measured_value = (
            measured_value[0] / self.frame_size[0],
            measured_value[1] / self.frame_size[1]
        )

        # 计算横向和纵向误差
        error_x = self.setpoint[0] - nomalized_measured_value[0]
        error_y = self.setpoint[1] - nomalized_measured_value[1]

        # 应用死区
        if abs(error_x) < self.dead_zone:
            error_x = 0
        if abs(error_y) < self.dead_zone:
            error_y = 0

        # PID 输出计算
        self.integral_x += error_x
        derivative_x = error_x - self.previous_error_x
        output_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x

        self.integral_y += error_y
        derivative_y = error_y - self.previous_error_y
        output_y = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y

        self.previous_error_x = error_x
        self.previous_error_y = error_y

        return output_x, output_y

    def move(self, output_x, output_y):
        pitch_now, yaw_now = Tripod_passer.get_loc()
        yaw_delta = output_x * self.max_yaw_angle
        pitch_delta = output_y * self.max_pitch_angle
        Tripod_passer.push_loc(pitch_delta+pitch_now, yaw_delta+yaw_now)
        
# -------------------------------
# PnP求解姿态
# -------------------------------
def solve_pnp(points_3D, points_2D, K_0, dist=None):
    points_3D = np.asarray(points_3D, dtype=np.float32)
    points_2D = np.asarray(points_2D, dtype=np.float32)
    if dist is None:
        dist = np.zeros((4, 1))  # 无畸变
    success, rvec, tvec = cv2.solvePnP(
        objectPoints=points_3D,
        imagePoints=points_2D,
        cameraMatrix=K_0,
        distCoeffs=dist,
        flags=cv2.SOLVEPNP_AP3P
    )
    return success, rvec, tvec

# # -------------------------------
# # 示例偏航角/俯仰角计算（根据tvec）
# # -------------------------------
def compute_yaw_pitch(tvec):
    x, y, z = tvec.flatten()
    yaw = math.degrees(math.atan2(x, z))
    pitch = math.degrees(math.atan2(-y, math.sqrt(x**2 + z**2)))
    return yaw, pitch

class track_process(object):
    def __init__(self,target_color):
        self.model_car, self.model_pose = network_init(weights_car=NET_PATH_CAR, weights_pose_armor=NET_PATH_POSE)
        self._matrix, self._dist_coeffs = read_yaml()
        print(self._matrix)
        self.target_color = target_color  # 或 'blue'
        self.priority_order = priority_order = [f'R{track_id}'] if reverse else [f'B{track_id}']    
        self.trackers = {}
        self.finding_flag = 1

    def finding(self):

        pitch ,yaw  = Tripod_passer.get_loc()
        new_yaw = yaw+self.finding_flag
        if new_yaw > 29:
            self.finding_flag = -0.1
        if new_yaw < -29:
            self.finding_flag = 0.1
        Tripod_passer.push_loc(pitch, new_yaw)

    def track(self):
        # global track_queue
        print("track start")
        finding_last = time.time()
        while True:
            if not track_queue.empty():
                frame0 = track_queue.get()
                flag0 = True
            else:
                flag0 = False
            if flag0:
                frame = frame0.copy()
                results = run_detect(frame, self.model_car, self.model_pose, target_color=self.target_color)
                
                # 目标跟踪和 PID 控制
                for priority_id in self.priority_order:
                    is_RS_detected = False
                    for result in results:

                        if result.id != priority_id:
                            continue
                        is_RS_detected = True      
                        x1, y1, x2, y2 = result.x1, result.y1, result.x2, result.y2 # bouning box
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)# car point

                        # 绘制目标框
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # 绿色框
                        cv2.putText(frame, f"{result.id}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        # 绘制装甲板框
                        cv2.rectangle(frame, (int(result.armor_x1), int(result.armor_y1)), 
                                    (int(result.armor_x2), int(result.armor_y2)), (255, 255, 0), 2)
                        cv2.putText(frame, f"{result.id}", (int(result.armor_x1), int(result.armor_y1) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        # 绘制测量点 (目标框中心)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                        # Create a new tracker if it doesn't exist
                        if result.id not in self.trackers:
                            self.trackers[result.id] = CarTracker(result.id, (cx, cy), [x1, y1, x2, y2])
                        
                        # 这里是卡尔曼滤波器
                        tracker = self.trackers[result.id]
                        tracker.car_point = (cx, cy) 
                        tracker.car_box = [x1, y1, x2, y2]
                        tracker.new_position = tracker.update_position()
                        cv2.circle(frame, tracker.new_position, 5, (255, 255, 255), -1)  # 表示预测点

                        # PID 控制器调整云台
                        pid_controller = PIDController(K_x=[0.1, 0.01, 0.05], K_y=[0.1, 0.01, 0.05], 
                                                    frame_size=(frame.shape[1]//2, frame.shape[0]//2))
                        output_x, output_y = pid_controller.update(tracker.new_position)
                        pid_controller.move(output_x, output_y)   

                        # Solve PnP
                        points_3D = np.array([[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]], dtype=np.float32) ## TODO: you can add it into config.py
                        tracker.keypoints = result.keypoints
                        print(tracker.keypoints)
                        K_0 = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)# TODO: replace it with camera matrix
                        _, rvec, tvec = solve_pnp(points_3D, tracker.keypoints , self._matrix,self._dist_coeffs)
                        # _, rvec, tvec = solve_pnp(points_3D, tracker.keypoints ,K_0)
                        print(tracker.keypoints)
                        
                        # TODO: Use the rvec and tvec to caculate location to localize the target
                        
                    
                    # missed_frame Processing
                    if not is_RS_detected and priority_id in self.trackers:
                        print(f"\033[31m没有检测到目标,使用filter位置\033[0m")
                        tracker = self.trackers[priority_id]
                        tracker.loss_frame()
                        tracker.polynomial_trend_predict() # 如果没有检测到目标，使用filter位置
                        tracker.missed_pred_point = tracker.guess_position()
                        print("guess")
                        tracker.calculate_box()
                        print(f"222")
                        tracker.calculate_keypoints()
                        print(f"目标丢失{tracker.missed_frames}帧，ID: {tracker.car_id}")
                        # 绘制预测点 (卡尔曼滤波器预测的中心)
                        cv2.circle(frame, tracker.missed_pred_point, 5, (0, 0, 255), -1)
                        print(f"111")
                        cv2.rectangle(frame, (int(tracker.missed_car_box[0]),int(tracker.missed_car_box[2])),
                                        (int(tracker.missed_car_box[1]),int(tracker.missed_car_box[3])), (0, 255, 0), 2)
                        print(f"111")
                        # Solve PnP
                        _, rvec, tvec = solve_pnp(points_3D, tracker.missed_keypoints , self._matrix,self._dist_coeffs)
                        # _, rvec, tvec = solve_pnp(points_3D, tracker.missed_keypoints ,K_0)
                        print(f"111")
                        # 根据 PID 控制器的输出调节云台（这里只是显示输出结果，实际应用中会控制云台的硬件）
                        output_x, output_y = pid_controller.update(tracker.new_position)
                        
                        pid_controller.move(output_x, output_y)   
                        print(f"\033[31m[PID INFO]\033[0mPID miseed_Control - X: {output_x}, Y: {output_y}")
                    # 如果该目标丢失超过最大帧数(10)，清除该追踪器
                    if (priority_id in self.trackers and self.trackers[priority_id].missed_frames > 10) or len(self.trackers) ==0:
                        # print(f"\033[33m目标 {priority_id} 丢失超过10帧，已移除追踪器。\033[0m")
                        if priority_id in self.trackers:
                            del self.trackers[priority_id] 
                        if time.time() - finding_last > 0.005:
                            self.finding()
                            finding_last = time.time()

                    
                if not track_result_queue.full():
                    track_result_queue.put(frame)   

# ------------------------------- 
# 主程序入口
# -------------------------------
if __name__ == "__main__" and True: 
    try:
        import serial,pexpect
        password = 'helloworld'
        TRIPOD_USB = '/dev/ttyUSB0'
        ch = pexpect.spawn('sudo chmod 777 {}'.format(TRIPOD_USB))
        ch.sendline(password)
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        _thread.start_new_thread(tripod_ser_send,(ser,))
        target_color = 'blue'  # 或 'blue'
        main_process = track_process(target_color=target_color)
        _thread.start_new_thread(track_pub_img,(Camera_Thread(0),))

        main_process.track()

    except Exception as e:
        print(f"[ERROR]{e}")

if __name__ == "__main__" and False:
    model_car, model_pose = network_init(weights_car='model/car_v11m_train10006_fp32.engine',
                                               weights_pose_armor='model/pose_best.engine')
    # the camera can only track one car at a time, so we add priority to help it make decision.
    target_color = 'red'  # 或 'blue'
    priority_order = ['RS'] if target_color == 'red' else ['BS']
    cap = cv2.VideoCapture("/home/helloworld/xsw/radar2025/video.mp4")

    trackers = {}  # 保存每个 ID 的 CarTracker
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧")
            break

        results = run_detect(frame, model_car,model_pose, target_color=target_color)
        
       # 目标跟踪和 PID 控制
        for priority_id in priority_order:
            is_RS_detected = False
            for result in results:

                if result.id != priority_id:
                    continue
                is_RS_detected = True      
                x1, y1, x2, y2 = result.x1, result.y1, result.x2, result.y2 # bouning box
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)# car point

                # 绘制目标框
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)  # 绿色框
                cv2.putText(frame, f"{result.id}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # 绘制装甲板框
                cv2.rectangle(frame, (int(result.armor_x1), int(result.armor_y1)), 
                            (int(result.armor_x2), int(result.armor_y2)), (255, 255, 0), 2)
                cv2.putText(frame, f"{result.id}", (int(result.armor_x1), int(result.armor_y1) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # 绘制测量点 (目标框中心)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Create a new tracker if it doesn't exist
                if result.id not in trackers:
                    trackers[result.id] = CarTracker(result.id, (cx, cy), [x1, y1, x2, y2])
                
                # 这里是卡尔曼滤波器
                tracker = trackers[result.id]
                tracker.car_point = (cx, cy) 
                tracker.car_box = [x1, y1, x2, y2]
                tracker.new_position = tracker.update_position()
                cv2.circle(frame, tracker.new_position, 5, (255, 255, 255), -1)  # 表示预测点

                # PID 控制器调整云台
                pid_controller = PIDController(K_x=[0.1, 0.01, 0.05], K_y=[0.1, 0.01, 0.05], 
                                               frame_size=(frame.shape[1]//2, frame.shape[0]//2))
                output_x, output_y = pid_controller.update(tracker.new_position)
                pid_controller.move(output_x, output_y)   

                # Solve PnP
                points_3D = np.array([[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]], dtype=np.float32) ## TODO: you can add it into config.py
                tracker.keypoints = result.keypoints
                K_0 = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)# TODO: replace it with camera matrix
                _, rvec, tvec = solve_pnp(points_3D, tracker.keypoints , K_0)
                print(tracker.keypoints)
                
                # TODO: Use the rvec and tvec to caculate location to localize the target
                
            
            # missed_frame Processing
            if not is_RS_detected and priority_id in trackers:
                print(f"\033[31m没有检测到目标,使用filter位置\033[0m")
                tracker = trackers['RS']
                tracker.loss_frame()
                tracker.polynomial_trend_predict() # 如果没有检测到目标，使用filter位置
                tracker.missed_pred_point = tracker.guess_position()
                tracker.calculate_box()
                tracker.calculate_keypoints()
                # 绘制预测点 (卡尔曼滤波器预测的中心)
                cv2.circle(frame, tracker.missed_pred_point, 5, (0, 0, 255), -1)
                cv2.rectangle(frame, (int(tracker.missed_car_box[0]),int(tracker.missed_car_box[2])),
                                (int(tracker.missed_car_box[1]),int(tracker.missed_car_box[3])), (0, 255, 0), 2)
                # Solve PnP
                _, rvec, tvec = solve_pnp(points_3D, tracker.missed_keypoints , K_0)
                # 根据 PID 控制器的输出调节云台（这里只是显示输出结果，实际应用中会控制云台的硬件）
                output_x, output_y = pid_controller.update(tracker.new_position)
                pid_controller.move(output_x, output_y)   
                print(f"\033[31m[PID INFO]\033[0mPID miseed_Control - X: {output_x}, Y: {output_y}")
            # 如果该目标丢失超过最大帧数(10)，清除该追踪器
            if priority_id in trackers and trackers[priority_id].missed_frames > 10:
                print(f"\033[33m目标 {priority_id} 丢失超过10帧，已移除追踪器。\033[0m")
                del trackers[priority_id]
            


        cv2.namedWindow("YOLOv8 Pose Prediction", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLOv8 Pose Prediction", 680, 400)
        cv2.imshow("YOLOv8 Pose Prediction", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
