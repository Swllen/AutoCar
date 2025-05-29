import os
import cv2
import time
from ultralytics import YOLO

# ===== 配置参数 =====
engine_path = "car_fp16.engine"
image_dir = "images"               # 你的图片文件夹路径
device = "cuda:0"
conf_thres = 0.25
model_predict_info = False         # 是否打印每张预测详情

# ===== 加载模型 =====
print(f"Loading model: {engine_path}")
model = YOLO(engine_path, task="detect", device=device)

# ===== 获取图片列表 =====
image_files = [f for f in os.listdir(image_dir) if f.lower().endswith((".jpg", ".jpeg", ".png"))]
print(f"Found {len(image_files)} images in {image_dir}")

# ===== 开始逐张推理 =====
total_time = 0
for i, filename in enumerate(image_files):
    path = os.path.join(image_dir, filename)
    image = cv2.imread(path)
    if image is None:
        print(f"[WARN] Skipping unreadable image: {filename}")
        continue

    start = time.time()
    results = model.predict(image, conf=conf_thres, augment=False, visualize=False, verbose=model_predict_info)
    end = time.time()

    elapsed = (end - start) * 1000  # 毫秒
    total_time += elapsed
    print(f"[{i+1}/{len(image_files)}] {filename}: {elapsed:.2f} ms")

# ===== 汇总平均耗时 =====
avg_time = total_time / len(image_files) if image_files else 0
print(f"\n✅ 推理完成！平均耗时: {avg_time:.2f} ms/张，模型: {engine_path}")
