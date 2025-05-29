import pexpect
import threading
import time
import serial
from serial_package.serial_api import *
class UART:
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

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            if self.debug:
                print(f"[UART] Closed {self.port}")

    def __del__(self):
        self.close()

if __name__ == "__main__":
    # 初始化 UART
    uart = UART(port="/dev/ttyACM0", baudrate=230400, password="stxin.789.", debug=True)

    # 构造并发送一条前进命令
    packet = forward(0.3)  # 0.3 m/s 前进
    uart.send_packet(packet)

    # 等待接收或保持主线程
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("退出中...")

    uart.close()