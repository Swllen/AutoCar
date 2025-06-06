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
        # self.kf.Q = np.diag([0.1, 0.1, 1, 1, 2.5, 2.5])
        self.kf.Q = np.diag([2.0, 2.0, 3.0, 3.0, 5.0, 5.0])

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

def update_yaw_pitch(pred_x, pred_y):
        nomalized_measured_value = (
            pred_x / 640,
            pred_y / 480
        )


        # 计算横向和纵向误差
        error_x = nomalized_measured_value[0] - 0.5
        error_y = nomalized_measured_value[1] - 0.5

        # # 应用死区
        # if abs(error_x) < 0.05:
        #     error_x = 0
        # if abs(error_y) < 0.05:
        #     error_y = 0

        yaw_delta = error_x * 15
        pitch_delta = error_y * 12
        
        return yaw_delta,pitch_delta
