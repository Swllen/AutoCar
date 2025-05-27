import os
import sys
import numpy as np
import cv2
from ctypes import *
from threading import Lock
import time
import queue
from datetime import datetime
import _thread


def open_camera(camera_type,is_init):
    cam = None
    init_flag = False
    try:
        cam = cv2.VideoCapture(camera_type)
        is_init = True
    except Exception as e:
        cam.release()
        print("[ERROR]:Open Camera Error")
    return is_init, cam

class Camera_Thread(object):
    def __init__(self, camera_type, video=False, video_path=None):
        '''
        the Camera_Thread class which will restart camera when it broken

        :param camera_type: 相机编号
        :param video: 是否使用视频
        :param video_path: 视频路径

        '''
        self._camera_type = camera_type
        self._open = False
        self._cap = None
        self._is_init = False
        self._video = video  
        self._video_path = video_path
        self._lock = Lock()  # 锁保护对象访问
        # try to open it once
        self.open()

    def open(self):
        # if camera not open, try to open it
        if not self._video:
            if not self._open:
                self._open, self._cap = open_camera(self._camera_type, self._is_init)
                if not self._is_init and self._open:
                    self._is_init = True
        else:
            if not self._open:
                self._cap = cv2.VideoCapture(self._video_path)
                self._open = True
                if not self._is_init and self._open:
                    self._is_init = True
    def is_open(self):
        '''
        check the camera opening state
        '''
        return self._open
    
    def read(self):
        if self._open:
            r, frame = self._cap.read()
            if not r:
                self._cap.release()  # release the failed camera
                self._open = False
            return r, frame
        else:
            return False, None
        

    def release(self):
        if self._open:
            self._cap.release()
            self._open = False

    def __del__(self):
        if self._open:
            self._cap.release()
            self._open = False


queue_size = 10
image_queue = queue.Queue(queue_size)

def pub_image(_cap):
    global image_queue
    while True:
        if _cap._open:
            _cap._lock.acquire()
            r, frame = _cap.read()
            _cap._lock.release()
            if r:  # 确保成功读取到帧
                if not image_queue.full():
                    image_queue.put(frame)
            else:
                print("Failed to get a valid frame.")
        else:
            print("The video source is not open.")
        time.sleep(0.02)  # 适当降低帧率，确保稳定

def save_img(_cap):
    # 用全局变量的话可能主程序来不及修改
    save_address = 'images/camera_test/'
    # 找不到录制文件夹就终止线程
    if not os.path.exists(save_address):
        print("The recording folder does not exist.")
        sys.exit()
    title = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    # name = os.path.join(save_address, title, 'camera0.jpg')
    name = save_address + title + '.jpg'
    _cap._lock.acquire()
    r, frame = _cap.read()
    _cap._lock.release()
    if r:
        cv2.imwrite(name, frame)
        print("Save the image successfully.")
    
def middle_record(_cap):
    save_address = 'images/camera_test/'
    os.makedirs(save_address, exist_ok=True)

    title = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    name = os.path.join(save_address, title + '.avi')

    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')  # 通用编码器
    record_fps = 25

    _cap._lock.acquire()
    ret, frame = _cap.read()
    _cap._lock.release()
    if not ret:
        print("[ERROR] 无法读取帧，录像中断")
        return

    h, w = frame.shape[:2]
    record_object = cv2.VideoWriter(name, fourcc, record_fps, (w, h))

    print(f"正在录制到文件：{name}")
    while True:
        _cap._lock.acquire()
        flag, frame = _cap.read()
        _cap._lock.release()
        if flag:
            record_object.write(frame)
        time.sleep(1.0 / record_fps)


def read_yaml(camera_type):
    # 读取相机标定参数
    fs = cv2.FileStorage("images/New_Folder/calibration_params.yaml", cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    distortion_coefficients = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, distortion_coefficients

if __name__ == "__main__":
    try:
        cam = Camera_Thread(0)
        _thread.start_new_thread(pub_image,(cam,))
        
        while True:
            close_flag = False
            if not image_queue.empty():
                frame0 = image_queue.get()
                flag0 = True
            else:
                flag0 = False
            if flag0:
                cv2.imshow("camera", frame0)
            k = cv2.waitKey(1)
            if k == ord('q'):
                close_flag = True
                print("close the camera")
            if k == ord('s'):
                _thread.start_new_thread(save_img,(cam,))
                print("save the image")
            if k == ord('r'):
                _thread.start_new_thread(middle_record,(cam,))
                print("start recording")
            if close_flag:
                break
    except Exception as e:
        print("[ERROR]{0}".format(e))
                
