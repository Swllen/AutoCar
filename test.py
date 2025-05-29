import cv2
import time
import sys

def open_camera(index=0):
    print(f"[INFO] Trying to open camera {index}...")
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print("[ERROR] Camera is occupied or unavailable.")
        return None
    print("[INFO] Camera opened successfully.")
    return cap

def release_camera(cap):
    if cap is not None and cap.isOpened():
        cap.release()
        print("[INFO] Camera released.")
    else:
        print("[INFO] No camera to release.")

def main():
    cap = open_camera(0)

    if cap is None:
        sys.exit("[EXIT] Cannot access camera.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame.")
            break
        cv2.imshow("Live Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[INFO] Quitting...")
            break

    release_camera(cap)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[INTERRUPT] Manually interrupted.")
        cv2.destroyAllWindows()
