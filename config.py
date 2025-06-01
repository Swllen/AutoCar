import queue

YAW_SERVO_ID = 0
PITCH_SERVO_ID = 1
USERVO_PORT = '/dev/ttyUSB0'  # Serial port for the servo manager
USERVO_BAUDRATE = 115200  # Baud rate for the servo manager
USERVO_TIMEOUT = 1  # Timeout for serial communication

CAR_PORT = '/dev/ttyACM0'  # Serial port for the car control
CAR_BOUDRATE = 230400  # Baud rate for the car control
PASSWORD = 'stxin.789.'  # Password for granting permission to access the serial port

DEBUG = True  # Debug mode for additional logging
LOG_DIR = './Logger/logs'  # Directory for log files

IMAGE_QUEUE_SIZE = 10  # Size of the image queue for camera frames
REMOTE_IMAGE_QUEUE=queue.Queue(maxsize=IMAGE_QUEUE_SIZE)

GET_TARGET = False
WIN_SIGNAL = False

NET_PATH = '/home/swllen/yolo/Car/yolov5-v7.0'