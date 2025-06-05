from collections import deque
import numpy as np
from filterpy.kalman import KalmanFilter
from control.Control import *

class ArmorKalmanFilter:
    def __init__(self, dt=1.0, q_scale=1.0, r_scale=1.0):
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

    def update_dt(self, dt):
        self.kf.F = np.array([
            [1, 0, dt, 0, 0.5 * dt ** 2, 0],
            [0, 1, 0, dt, 0, 0.5 * dt ** 2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

    def update(self, point, dt=None):
        if point is None:
            return None
        if dt is not None:
            self.update_dt(dt)

        z = np.array([point[0], point[1]])
        if not self.initialized:
            self.kf.x[:2] = z.reshape((2, 1))
            self.kf.x[2:] = 0
            self.initialized = True

        self.kf.predict()
        self.kf.update(z)
        return self.kf.x.copy()


class CarTracker:
    def __init__(self, car_point, car_box, dt=1.0):
        self.car_point = car_point
        self.car_box = car_box
        self.pred_position = None
        self.kf = ArmorKalmanFilter(dt)
        self.missed_frames = 0
        self.history = deque(maxlen=5)
        self.missed_car_point = None
        self.missed_pred_point = None
        self.kf_state = []
        
    def update_position(self, dt=None):
        self.history.append(self.car_point)
        self.missed_frames = 0
        self.kf_state = self.kf.update(self.car_point, 1)
        self.pred_position = (int(self.kf.kf.x[0, 0]), int(self.kf.kf.x[1, 0]))
        return self.kf_state.copy()

    def polynomial_trend_predict(self, order=1):
        if len(self.history) < order + 1:
            self.missed_car_point = self.car_point
            return
        x = list(range(len(self.history)))
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]
        coef_x = np.polyfit(x, xs, order)
        coef_y = np.polyfit(x, ys, order)
        next_idx = len(self.history)
        pred_x = int(np.polyval(coef_x, next_idx))
        pred_y = int(np.polyval(coef_y, next_idx))
        self.missed_car_point = (pred_x, pred_y)

    def loss_frame(self):
        self.missed_frames += 1

    def calculate_missed_box(self):
        x_offset = (self.car_box[2] - self.car_box[0]) / 2
        y_offset = (self.car_box[3] - self.car_box[1]) / 2
        self.missed_car_box = [
            self.missed_car_point[0] - x_offset, self.missed_car_point[0] + x_offset,
            self.missed_car_point[1] - y_offset, self.missed_car_point[1] + y_offset
        ]

class PIDController:
    def __init__(self, K_x, K_y, uservo:UservoController, setpoint=(0.5, 0.5), frame_size=[640, 480], dead_zone=0.00):
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
        error_x = self.setpoint[0] - nomalized_measured_value[0]
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
        if DEBUG:
            logger.debug(f"[PID] messured_x: {nomalized_measured_value[0]:.4f}, messured_y: {nomalized_measured_value[1]:.4f}")
            logger.debug(f"[PID] Error X: {error_x:.4f}, Error Y: {error_y:.4f}")
            logger.debug(f"[PID] Output X: {output_x:.4f}, Output Y: {output_y:.4f}")

        return output_x, output_y

    def move(self, output_x, output_y):
        yaw_now, pitch_now = self.uservo.get_yaw(), self.uservo.get_pitch()
        yaw_delta = output_x * self.max_yaw_angle
        pitch_delta = output_y * self.max_pitch_angle
        self.uservo.set_yaw(yaw_delta+yaw_now)
        self.uservo.set_pitch(pitch_delta+pitch_now)
        if DEBUG:
            logger.debug(f"[MOVE] Pitch Now: {pitch_now:.3f}, Yaw Now: {yaw_now:.3f}")
            logger.debug(f"[MOVE] Pitch Delta: {pitch_delta:.4f}, Yaw Delta: {yaw_delta:.4f}")
