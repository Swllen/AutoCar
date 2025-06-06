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



class UservoSer:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1, password=None,
                 debug=True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.password = password
        self.debug = debug
        self.ser = None
        self._ser_init = False
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
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,
                                     parity=serial.PARITY_NONE, stopbits=1,
                                     bytesize=8, timeout=0)
            if self.ser.is_open:
                logger.info(f"Opened {self.port}")
                self._ser_init = True
        except Exception as e:
            logger.error(f"Could not open port {self.port}: {e}")
            self.ser = None

    def send_packet(self, yaw, pitch, flag):
        if self._ser_init:
            if flag == 0:
                flag = 1
            else:
                flag = 0
            data = build_yaw_pitch_package(yaw,pitch,flag)
            self.ser.write(data)
            logger.info(f"Yaw {yaw}° ,Pitch {pitch}° is published. Cruise Flag is {flag}")
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

    def beep_on(self):
        self.send_packet(build_beep_on_frame())
    
    def beep_off(self):
        self.send_packet(build_beep_off_frame())

    def stop(self):
        self.send_packet(stop())
    
    def victory(self):
        print("Victory signal started")
        self.beep_on()
        time.sleep(0.25)
        self.beep_off()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info(f"Closed {self.port}")

    def __del__(self):
        self.close()


if __name__ == "__main__":
    try:
        uservo_ser = UservoSer(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)
        t1 = time.time()
        uservo_ser.send_packet(5.1,2,1)
        t2 = time.time()
        print(1/(t2-t1))

        car = RobotController(port=CAR_PORT, password=PASSWORD, baudrate=CAR_BAUDRATE, debug=DEBUG)
        # car.beep_on()
        # time.sleep(1)
        car.beep_off()
        
        # # time.sleep(1)
        # # # car.stop()
        # # # speed = random.uniform(-1, 1)
        # # # single_time = random.uniform(0.5, 2)
        # # speed = 0.5
        # # car.motion(90,speed)
        # # time.sleep(0.2)
        car.stop()
        # # car.__del__()  
    except KeyboardInterrupt:
        logger.info("退出中...")
    finally:
        logger.info("程序结束")
