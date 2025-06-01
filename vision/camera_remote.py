from flask import Flask, Response, render_template_string
import cv2
from Logger.Logger import logger
from config import WIN_SIGNAL, GET_TARGET
app = Flask(__name__)
remote_image_queue = None  # 由主程序注入

def set_queue(q):
    global remote_image_queue
    remote_image_queue = q

def generate():
    while True:
        if remote_image_queue is not None and not remote_image_queue.empty():
            frame = remote_image_queue.get()
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template_string('''
        <!doctype html>
        <html>
        <head>
            <title>Jetson Nano 实时摄像头</title>
            <style>
                body {
                    background-color: #f5f5f5;
                    font-family: Arial, sans-serif;
                    text-align: center;
                    margin: 0;
                    padding: 0;
                }
                .container {
                    padding-top: 30px;
                }
                .status {
                    margin-top: 20px;
                    font-size: 18px;
                    color: #333;
                }
                img {
                    border: 4px solid #444;
                    border-radius: 10px;
                    box-shadow: 0 0 15px rgba(0,0,0,0.2);
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h2>🎥 jetson Nano 实时摄像头</h2>
                <img src="/video" width="640" height="480" alt="Camera Stream">
                <div class="status">
                    <p>当前状态: <strong>{{ '目标检测中' if GET_TARGET else '未检测目标' }}</strong></p>
                    <p>WINSIGNAL: <strong>{{ '已开启' if WIN_SIGNAL else '未开启' }}</strong></p>
                </div>
            </div>
        </body>
        </html>
    ''')


@app.route('/video')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 启动 Flask 的线程
def run_flask():
    app.run(host='0.0.0.0', port=8080, threaded=True)
