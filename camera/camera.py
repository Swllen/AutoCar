import os
import sys
import cv2
import time
import queue
import threading
import camera.camera_remote as camera_remote
from datetime import datetime
from config import REMOTE_IMAGE_QUEUE

def open_camera(camera_type):
    try:
        cam = cv2.VideoCapture(camera_type)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cam.isOpened():
            raise Exception("Camera open failed")
        return True, cam
    except Exception as e:
        print("[ERROR]: Open Camera Error:", e)
        return False, None

class CameraThread:
    def __init__(self, camera_type, video=False, video_path=None):
        self._camera_type = camera_type
        self._video = video
        self._video_path = video_path
        self._cap = None
        self._open = False
        self._lock = threading.Lock()
        self.open()

    def open(self):
        if not self._video:
            if not self._open:
                self._open, self._cap = open_camera(self._camera_type)
        else:
            if not self._open:
                self._cap = cv2.VideoCapture(self._video_path)
                self._open = self._cap.isOpened()

    def is_open(self):
        return self._open

    def read(self):
        with self._lock:
            if self._open:
                ret, frame = self._cap.read()
                if not ret:
                    self._cap.release()
                    self._open = False
                    return False, None
                frame = cv2.flip(frame, 0)
                return ret, frame
            return False, None

    def release(self):
        with self._lock:
            if self._open:
                self._cap.release()
                self._open = False

    def __del__(self):
        self.release()

image_queue = queue.Queue(maxsize=10)

def pub_image(cap: CameraThread):
    while True:
        if cap.is_open():
            ret, frame = cap.read()
            if ret:
                if image_queue.full():
                    try:
                        image_queue.get_nowait()
                    except queue.Empty:
                        pass
                image_queue.put(frame)
            else:
                print("[WARN] Failed to read frame")
        else:
            print("[WARN] Camera not open")
        time.sleep(0.02)

def save_img(cap: CameraThread):
    save_dir = 'images/camera_test/'
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    path = os.path.join(save_dir, f"{timestamp}.jpg")
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(path, frame)
        print("Image saved:", path)
    else:
        print("[ERROR] Failed to capture image")

def middle_record(cap: CameraThread):
    save_dir = 'images/camera_test/'
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    filename = os.path.join(save_dir, f"{timestamp}.avi")

    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to read frame")
        return

    h, w = frame.shape[:2]
    fps = 25
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    writer = cv2.VideoWriter(filename, fourcc, fps, (w, h))
    print("Recording started:", filename)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Frame read failed, stopping recording")
            break
        writer.write(frame)
        time.sleep(1.0 / fps)

def main():
    camera_remote.set_queue(REMOTE_IMAGE_QUEUE)

    flask_thread = threading.Thread(target=camera_remote.run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    cam = CameraThread(0)
    t = threading.Thread(target=pub_image, args=(cam,), daemon=True)
    t.start()

    print("Press 's' to save image, 'r' to record, 'q' to quit")

    while True:
        start_time = time.time()
        frame = image_queue.get()
        REMOTE_IMAGE_QUEUE.put(frame)
        end_time = time.time()
        print(f"FPS:{1/(end_time - start_time):.4f}")
if __name__ == "__main__":
    main()
