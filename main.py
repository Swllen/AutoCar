from control.Control import *
from tracking.new_track import *
from inference.infer import *


class main_process:
    def __init__(self,car:RobotController=None,uservo:UservoSer=None,track:track_process=None):
        self.car = car
        self.uservo_ser = uservo
        self.track = track
    
    def move_car(self):
        # Start the self.car moving forward
        self.car.forward(1.5)
        time.sleep(1.3)
        self.car.stop()
        self.car.rotate_ccw(10)
        time.sleep(0.4)
        self.car.stop()
        # motion_thread = threading.Thread(target=self.car.back_and_forth, args=(0.5, 2.0), daemon=True)
        # motion_thread.start()
        # self.car.back_and_forth(5, 1.0)
        logger.info("Car is moving forward.")
        t1 = time.time()
        while t2-t1 < 60:
            self.car.motion(-0.25, 1)
            # logger.info(f"Moving forward at {speed_mps} m/s")
            time.sleep(1)
            self.car.stop()
            time.sleep(1)
            # logger.info("Stopped")
            # time.sleep(0.5)
            self.car.motion(3.14, 0.7)
            # logger.info(f"Moving backward at {speed_mps} m/s")
            time.sleep(1)
            self.car.stop()
            time.sleep(1)
            t2 = time.time()

if __name__ == "__main__":

    is_detected = 0  # 初始化为 0，表示没有检测到目标
    
    input_shape = (480, 640, 3)
    resized_shape = (3, 320, 320)

    input_array = Array(ctypes.c_uint8, int(np.prod(input_shape)), lock=False)
    output_array = Array(ctypes.c_float, int(np.prod(resized_shape)), lock=False)
    input_lock = Lock()
    output_lock = Lock()

    
    
    try:
        cam = CameraProcess(input_size=(640, 480), resize_size=(320, 320))
        cam.start(input_array, output_array, input_lock, output_lock)
        record_thread = threading.Thread(target=record,args=(input_array,input_lock,input_shape),daemon=True)
        record_thread.start()
        
        uservo = UservoSer(port=USERVO_PORT, password=PASSWORD, baudrate=USERVO_BAUDRATE, debug=DEBUG)
        car = RobotController(port=CAR_PORT, password=PASSWORD, baudrate=CAR_BAUDRATE, debug=DEBUG)
        track = track_process(model_path=NET_PATH,
                            uservo_ser=uservo,
                            car_controller = car,
                            output_array=output_array,
                            output_lock=output_lock,
                            input_array=input_array,
                            input_lock=input_lock,
                            input_shape=input_shape,
                            resized_shape=resized_shape)
        main = main_process(car=car,uservo=uservo,track=track)
        move_thread = threading.Thread(target=main.move_car)

        print("Press 's' to start")
        user_input = input().strip().lower()
        if user_input == 's':
            move_thread.start()
            main.track.track()
            
    except KeyboardInterrupt:
        print("Interrupted, stopping...")
    finally:
        if cam is not None:
            cam.stop()
            # if main.track.servo_thread is not None:
            #     main.track.servo_thread.stop()
            #     cv2.destroyAllWindows()
