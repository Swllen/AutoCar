import pexpect
import threading
import time
import serial
from serial_package.serial_api import *
from serial_package.uservo import UartServoManager
from ..config import *

class UservoController:
    def __init__(self,port="/dev/ttyUSB0", baudrate=115200, timeout=1, password=None, /
                 debug=True,yaw_servo_id=YAW_SERVO_ID, pitch_servo_id=PITCH_SERVO_ID):
        self.servos = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.password = password
        self.debug = debug
        self.ser = None
        self.yaw_servo_id = yaw_servo_id
        self.pitch_servo_id = pitch_servo_id
        self._lock = threading.Lock()
        self._init_serial()
        self._init_servos()
    def _grant_permission(self):
        if self.password:
            try:
                ch = pexpect.spawn(f'sudo chmod 777 {self.port}')
                ch.expect('password', timeout=2)
                ch.sendline(self.password)
                ch.expect(pexpect.EOF, timeout=2)
                if self.debug:
                    print(f"[UART] Permission granted for {self.port}")
            except Exception as e:
                print(f"[UART ERROR] Permission change failed: {e}")

    def _init_serial(self):
        self._grant_permission()
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,\
					 parity=serial.PARITY_NONE, stopbits=1,\
					 bytesize=8,timeout=0)
            if self.ser.is_open and self.debug:
                print(f"[UART] Opened {self.port}")
        except Exception as e:
            print(f"[UART ERROR] Could not open port {self.port}: {e}")
            self.ser = None
    def _init_servos(self):
        if self.ser is not None:
            self.servos = UartServoManager(self.ser, is_debug=self.debug)
            if self.debug:
                print("[UART] Servo manager initialized")
        else:
            print("[UART ERROR] Serial port not initialized, cannot create servo manager")
    def set_pitch(self,angle, interval=1000):
        if self.servos is not None:
            self.servos.set_servo_angle(self.pitch_servo_id, angle, interval=interval)
            self.servos.wait()
            if self.debug:
                print(f"[UART] Pitch servo set to {angle}°")
        else:
            print("[UART ERROR] Servo manager not initialized")
    def set_yaw(self, angle, interval=1000):
        if self.servos is not None:
            self.servos.set_servo_angle(self.yaw_servo_id, angle, interval=interval)
            self.servos.wait()
            if self.debug:
                print(f"[UART] Yaw servo set to {angle}°")
        else:
            print("[UART ERROR] Servo manager not initialized")
    def get_yaw(self):
        if self.servos is not None:
            yaw = self.servos.query_servo_angle(self.yaw_servo_id)
            if self.debug:
                print(f"[UART] Yaw servo current angle: {yaw}°")
            return yaw
        else:
            print("[UART ERROR] Servo manager not initialized")
            return None
    def get_pitch(self):
        if self.servos is not None:
            pitch = self.servos.query_servo_angle(self.pitch_servo_id)
            if self.debug:
                print(f"[UART] Pitch servo current angle: {pitch}°")
            return pitch
        else:
            print("[UART ERROR] Servo manager not initialized")
            return None

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
                if self.debug:
                    print(f"[UART] Permission granted for {self.port}")
            except Exception as e:
                print(f"[UART ERROR] Permission change failed: {e}")

    def _init_serial(self):
        self._grant_permission()
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.ser.is_open and self.debug:
                print(f"[UART] Opened {self.port}")
        except Exception as e:
            print(f"[UART ERROR] Could not open port {self.port}: {e}")
            self.ser = None

    def send_packet(self, data: bytes):
        if self.ser and self.ser.is_open:
            with self._lock:
                self.ser.write(data)
                if self.debug:
                    print(f"[UART TX] {data.hex(' ')}")

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
            if self.debug:
                print(f"[UART] Closed {self.port}")

    def __del__(self):
        self.close()

if __name__ == "__main__":
    # 初始化 UART
    car = RobotController(port='/dev/ttyUSB0', password='your_password', debug=True)
    car.forward(0.5)  # 前进0.5 m/s
    time.sleep(2)  # 前进2秒
    car.stop()  # 停止
    time.sleep(1)  # 停止1秒
    car.rotate_cw(1.0)  # 顺时针旋转1 rad/s
    time.sleep(2)  # 旋转2秒
    car.stop()  # 停止旋转  
    
    # 等待接收或保持主线程
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("退出中...")

    uart.close()