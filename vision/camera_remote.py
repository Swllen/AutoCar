from flask import Flask, Response, render_template_string
import cv2
from Logger.Logger import logger
app = Flask(__name__)
image_queue = None  # 由主程序注入

def set_queue(q):
    global image_queue
    image_queue = q

def generate():
    while True:
        if image_queue is not None and not image_queue.empty():
            frame = image_queue.get()
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template_string('''
        <html>
        <body>
            <h2>Jetson Nano 实时摄像头</h2>
            <img src="/video" width="640" height="480">
        </body>
        </html>
    ''')

@app.route('/video')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
