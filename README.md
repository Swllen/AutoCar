# AutoCar-光电设计竞赛智能激光对抗小车


### 如何快速上手
- 学会连接nano高效开发： 挂载文件方便开发，示例：`udo sshfs -o allow_other shitianxin@10.42.0.34:/home/shitianxin /mnt/nano`；远程ssh -X 连接可以端口转发用来可视化（一定要连接网线）
- 首先要知道此代码是按照模块开发，自己写了很多模块，在其他程序调用的时候都是需要`import xxx`或者`from xxx import xxx`，如果想要单纯运行某个模块，在根目录使用命令`python3 -m camera.camera`，运行最终的main.py文件就跟正常运行py文件的方法一样。
- 自己试着跑通`infer.py`,`new_track.py`等功能。
- 追踪参数调整在`car_tracker`里面调整参数就好
- 会使用主程序
- 看懂代码架构和逻辑

### Camera模块
-` setcamera.py`的功能是用来调整USB相机的参数，在运行别的程序的时候也可以运行，根据命令行提示可以修改相机相关的参数
- `camera_process.py`的功能就是打开相机取图，包括设置取图格式/相机录制线程/可视化函数。主要的是相机进程类，为了充分利用nano的CPU资源，提高推理并发性，相机取图之后直接做一个preprocess。
- 使用方法：`infer.py`的__main__有详情实例，学着使用即可。
- `TODO`:后续优化可以在相机打开之后把setcamera集成进去，这样子可以每次开主程序都能调参，避免忘记，或者写一个按键判断是否调整参数。
### Logger模块
- 用来记录日志信息，具体使用看代码
- TODO：要想优化的话就把程序其他部分的关键点加省`logger.info`，比如初始化之类的

### Inference模块
- 照搬`tensorrtx/yolov5/yolo_det_trt.py`的逻辑，增加了多进程处理，将预处理和推理分开提升运行速度，实测从25fps增加到75fps。
- 对推理的结果进行了筛选，每次只返回一个目标。
- 使用方法：本文件的__main__有详情实例，学着使用即可。如果要更换模型请查阅tensorrtx如何使用
- TODO：把`CONFTHREAD/IOU`等参数集成到`cofig.py`里面统一修改。

### Tracking模块
- car_tracker：定义了卡尔曼滤波器，cartracker类还有一些基本的计算函数，使用的时候只需要调整参数就好
- new_track.py：主要函数，实现追踪加通信下发
- TODO：将通信和主程序隔离开来，通信以高频率并行下发/增加丢帧预测处理（现在丢帧没有任何的处理行为）/优化胜利判断条件，写成多线程的格式。

### serial_package和Control模块
- serial_package主要功能为构建通信的packet，将数据打包为能通信下发的格式.
- control通过调用serial_pakage构建通信包，定义robot和uservo两个类来实现功能。

### 优化方向/TODO
- config.py增加参数优化，把一些适合统一调整的变量放进来方便管理
- 将通信和track主程序隔离开来，通信以高频率并行下发/增加丢帧预测处理（现在丢帧没有任何的处理行为）/优化胜利判断条件，写成多线程的格式。
- 后续优化可以在相机打开之后把setcamera集成进去，这样子可以每次开主程序都能调参，避免忘记，或者写一个按键判断是否调整参数。
- 把程序其他部分的关键点加logger.info，比如初始化之类的
- 把`CONFTHREAD/IOU`等参数集成到`cofig.py`里面统一修改。
- main.py运行优化，可以先把track打开但是胜利判断根据是否开始的flag来决定是否打开，这样在等待的时候也能追踪，按下按键小车开始动。
- cruise逻辑优化。
- 角度计算优化：采取相机内参矩阵的方式
- 机械结构优化，换轮子，或者增加减震功能，设计一下上层结构。


