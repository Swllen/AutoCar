import pexpect
import threading
import time
import random

from Logger.Logger import logger
from serial_package.serial_api import *
from serial_package.uservo import UartServoManager
from config import *
import serial
import struct


class UservoController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1, password=None,
                 debug=True, yaw_servo_id=YAW_SERVO_ID, pitch_servo_id=PITCH_SERVO_ID):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.password = password
        self.debug = debug
        self.yaw_servo_id = yaw_servo_id
        self.pitch_servo_id = pitch_servo_id
        self.ser = None
        self._lock = threading.Lock()
        self.servos = None
        self._servos_init = False
        self._init_serial()
        self._init_servos()
        self.cruise_step = 3  # Default cruise step for yaw servo
    def _grant_permission(self):
        if self.password:
            try:
                ch = pexpect.spawn(f'sudo chmod 777 {self.port}')
                ch.expect('password', timeout=2)
                ch.sendline(self.password)
                ch.expect(pexpect.EOF, timeout=2)
                logger.info(f"Permission granted for {self.port}")
            except Exception as e:
                logger.error(f"Permission change failed: {e}")

    def _init_serial(self):
        self._grant_permission()
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,
                                     parity=serial.PARITY_NONE, stopbits=1,
                                     bytesize=8, timeout=0)
            if self.ser.is_open:
                logger.info(f"Opened {self.port}")
        except Exception as e:
            logger.error(f"Could not open port {self.port}: {e}")
            self.ser = None

    def _init_servos(self):
        if self.ser:
            try:
                self.servos = UartServoManager(self.ser, is_debug=self.debug)
                self._servos_init = True
                logger.info("Servo manager initialized")
            except Exception as e:
                logger.error(f"Failed to initialize servo manager: {e}")
                self.servos = None
                self._servos_init = False
        else:
            logger.error("Serial port not initialized, cannot create servo manager")
    # y-direction, set angle
    def set_pitch(self, angle, interval=0, velocity=10):
        if self._servos_init:
            self.servos.set_servo_angle(self.pitch_servo_id, angle, interval=interval,velocity=velocity)
            # self.servos.wait()
            logger.info(f"Pitch servo set to {angle}°")
        else:
            logger.error("Servo manager not initialized")
    # x-direction, set angle
    def set_yaw(self, angle, interval=0, velocity=1000):
        if self._servos_init:
            self.servos.set_servo_angle(self.yaw_servo_id, angle, interval=interval)
            # self.servos.wait()
            logger.info(f"Yaw servo set to {angle}°")
        else:
            logger.error("Servo manager not initialized")
    # y-direction, get angle
    def get_pitch(self):
        if self._servos_init:
            pitch = self.servos.query_servo_angle(self.pitch_servo_id)
            logger.info(f"Pitch servo current angle: {pitch}°")
            return pitch
        logger.error("Servo manager not initialized")
        return None
    # x-direction, get angle
    def get_yaw(self):
        if self._servos_init:
            yaw = self.servos.query_servo_angle(self.yaw_servo_id)
            logger.info(f"Yaw servo current angle: {yaw}°")
            return yaw
        logger.error("Servo manager not initialized")
        return None
    
    def cruise(self):
        if self._servos_init:
            yaw = self.get_yaw()
            new_yaw = yaw + self.cruise_step
            if new_yaw > 90:
                self.cruise_step = - self.cruise_step
            if new_yaw < -90:
                self.cruise_step = - self.cruise_step
            self.set_yaw(new_yaw)
        else:
            logger.error("Servo manager not initialized")


class RobotController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1, password=None, debug=False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.password = password
        self.debug = debug
        self.ser = None
        self._lock = threading.Lock()
        self._init_serial()

    def _grant_permission(self):
        if self.password:
            try:
                ch = pexpect.spawn(f'sudo chmod 777 {self.port}')
                ch.expect('password', timeout=2)
                ch.sendline(self.password)
                ch.expect(pexpect.EOF, timeout=2)
                logger.info(f"Permission granted for {self.port}")
            except Exception as e:
                logger.error(f"Permission change failed: {e}")

    def _init_serial(self):
        self._grant_permission()
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.ser.is_open:
                logger.info(f"Opened {self.port}")
        except Exception as e:
            logger.error(f"Could not open port {self.port}: {e}")
            self.ser = None

    def send_packet(self, data: bytes):
        if self.ser and self.ser.is_open:
            with self._lock:
                self.ser.write(data)
                # logger.debug(f"[TX] {data.hex(' ')}")

    def read_all(self):
        if self.ser and self.ser.is_open:
            with self._lock:
                return self.ser.read_all()
        return b''

    def start_receive_loop(self, callback):
        def loop():
            buffer = b''
            while True:
                buffer += self.read_all()
                if buffer:
                    callback(buffer)
                    buffer = b''
                time.sleep(0.05)
        threading.Thread(target=loop, daemon=True).start()

    def forward(self, speed_mps):
        self.send_packet(forward(speed_mps))

    def backward(self, speed_mps):
        self.send_packet(backward(speed_mps))
    def motion(self, angle, speed_mps):
        self.send_packet(motion(angle, speed_mps))
    def rotate_cw(self, w_rads):
        self.send_packet(rotate_cw(w_rads))

    def rotate_ccw(self, w_rads):
        self.send_packet(rotate_ccw(w_rads))

    def turn_left(self, speed_mps, turn_ratio):
        self.send_packet(turn_left(speed_mps, turn_ratio))

    def turn_right(self, speed_mps, turn_ratio):
        self.send_packet(turn_right(speed_mps, turn_ratio))

    def stop(self):
        self.send_packet(stop())

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info(f"Closed {self.port}")

    def __del__(self):
        self.close()


class UservoThread(threading.Thread):
    def __init__(self, uservo_controller: UservoController, frequency=FREQUENCE, alpha=0.4):
        super().__init__()
        self.uservo = uservo_controller
        self.freq = frequency
        self.alpha = alpha
        self.interval = 1.0 / self.freq

        self.target_yaw = 0
        self.target_pitch = 0
        self.current_yaw = 0
        self.current_pitch = 0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def set_target(self, yaw, pitch):
        with self._lock:
            self.target_yaw = yaw
            self.target_pitch = pitch

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            with self._lock:
                # 插值
                t1 = time.time()
                self.current_yaw = (1 - self.alpha) * self.current_yaw + self.alpha * self.target_yaw
                self.current_pitch = (1 - self.alpha) * self.current_pitch + self.alpha * self.target_pitch

                # 发命令
                self.uservo.set_yaw(self.current_yaw)
                self.uservo.set_pitch(self.current_pitch)
                t2 = time.time()
                waste_time = t2 - t1
            time.sleep(self.interval - waste_time)

if __name__ == "__main__":
    try:
        uservo = UservoController(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)
        # uservo.get_pitch()
        # uservo.get_yaw()
        t1 = time.time()
        uservo.set_yaw_pitch(45,0)
        # uservo.set_pitch(0)
        t2 = time.time()
        print(1/(t2-t1))
        # uservo.set_pitch(0)
        # uservo.set_yaw(0)
        car = RobotController(port=CAR_PORT, password=PASSWORD, baudrate=CAR_BAUDRATE, debug=DEBUG)
        car.forward(1.5)
        
        time.sleep(1.5)
        car.stop()
        car.turn_right(0.3,1.5)
        time.sleep(1)
        car.stop()
        # # # speed = random.uniform(-1, 1)
        # # # single_time = random.uniform(0.5, 2)
        # # speed = 0.5
        # # car.motion(90,speed)
        # # time.sleep(0.2)
        # # car.stop()
        # # car.__del__()
        detected = False
        while True:
            if not detected:
                uservo.cruise()
            else:
                uservo.set_yaw(0)    
    except KeyboardInterrupt:
        logger.info("退出中...")
    finally:
        logger.info("程序结束")
