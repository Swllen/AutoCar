import cv2
import threading
import queue
import vision.camera_remote as camera_remote
from Logger.Logger import logger

def camera_init():
        # 启动摄像头读取主循环
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()
    else:
        logger.info("Camera initialized successfully.")
    return cap

if __name__ == "__main__":
        # 初始化图像队列
    image_queue = queue.Queue(maxsize=10)
    camera_remote.set_queue(image_queue)

    flask_thread = threading.Thread(target=camera_remote.run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    cap = camera_init()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            continue

        # 处理图像（比如上下翻转）
        # frame = cv2.flip(frame, 0)

        # 将处理后图像放入队列
        if not image_queue.full():
            image_queue.put(frame)
