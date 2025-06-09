# AutoCar
光电竞赛激光对抗小车
# TODO LIST
- [x] nano刷系统
- [ ] nano环境配置
- [ ] 在nano上面部署tensorrt加速，看看yolov5s的模型能跑多少帧
- [ ] jetsonnano的GPIO输出协议
- [ ] 调研一下拓展板和jetsonnano的GPIO怎么通信使用，还有如何供电（感觉这个好乱）
- [ ] 雷达建模，跑通激光雷达
- [ ] 训练模型
/media/swllen/Extern2/CAR/AutoCar
├── camera
│   ├── camera_init.py
│   ├── camera.py
│   ├── camera_remote.py
│   ├── __pycache__
│   │   ├── camera.cpython-36.pyc
│   │   ├── camera_init.cpython-36.pyc
│   │   ├── camera_remote.cpython-36.pyc
│   │   └── setcamera.cpython-36.pyc
│   └── setcamera.py
├── config.py
├── control
│   ├── Control.py
│   └── __init__.py
├── images
│   └── camera_test
│       ├── 2025_05_27_00_30_33.mp4
│       ├── 2025_05_27_10_10_33.jpg
│       ├── 2025_05_27_10_11_20.mp4
│       ├── 2025_05_27_10_13_19.avi
│       ├── 2025_05_27_10_14_21.avi
│       ├── 2025_05_27_10_15_31.avi
│       ├── 2025_05_27_10_16_10.avi
│       └── 2025_05_27_10_19_11.avi
├── inference
│   ├── infer.py
│   └── __pycache__
│       └── infer.cpython-36.pyc
├── Logger
│   ├── __init__.py
│   ├── Logger.py
│   ├── logs
│   │   ├── 2025-05-31 16-05-19.txt
│   │   ├── 2025-05-31 16-27-13.txt
│   │   ├── 2025-05-31 16-27-39.txt
│   │   ├── 2025-05-31 16-29-16.txt
│   │   ├── 2025-05-31 16-38-35.txt
│   │   ├── 2025-05-31 16-39-09.txt
│   │   ├── 2025-05-31 16-39-36.txt
│   │   ├── 2025-05-31 16-40-02.txt
│   │   ├── 2025-05-31 16-42-21.txt
│   │   ├── 2025-05-31 18-06-33.txt
│   │   ├── 2025-05-31 18-07-19.txt
│   │   ├── 2025-05-31 18-10-21.txt
│   │   ├── 2025-05-31 18-11-18.txt
│   │   ├── 2025-05-31 18-19-27.txt
│   │   ├── 2025-05-31 18-21-24.txt
│   │   ├── 2025-05-31 18-21-39.txt
│   │   └── 2025-05-31 23-07-08.txt
│   └── __pycache__
│       ├── __init__.cpython-311.pyc
│       └── Logger.cpython-311.pyc
├── predict.py
├── __pycache__
│   └── config.cpython-311.pyc
├── README.md
├── serial_package
│   ├── __init__.py
│   ├── serial_api.py
│   └── uservo.py
├── test.py
├── tracking
│   ├── car_tracker.py
│   ├── new_track.py
│   ├── __pycache__
│   │   ├── car_tracker.cpython-36.pyc
│   │   ├── new_track.cpython-36.pyc
│   │   ├── track.cpython-36.pyc
│   │   └── track_test.cpython-36.pyc
│   ├── track.py
│   └── track_test.py
├── tree.md
├── vision
│   ├── camera_init.py
│   ├── camera.py
│   ├── camera_remote.py
│   ├── filter.py
│   ├── __init__.py
│   ├── predictor.py
│   ├── __pycache__
│   │   ├── camera.cpython-311.pyc
│   │   ├── camera_init.cpython-311.pyc
│   │   ├── camera_remote.cpython-311.pyc
│   │   └── __init__.cpython-311.pyc
│   ├── setcamera.py
│   └── track.py
└── yolov5
    ├── 0602.engine
    ├── best.engine
    ├── build
    │   ├── CMakeCache.txt
    │   ├── CMakeFiles
    │   │   ├── 3.13.0
    │   │   │   ├── CMakeCCompiler.cmake
    │   │   │   ├── CMakeCUDACompiler.cmake
    │   │   │   ├── CMakeCXXCompiler.cmake
    │   │   │   ├── CMakeDetermineCompilerABI_C.bin
    │   │   │   ├── CMakeDetermineCompilerABI_CUDA.bin
    │   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
    │   │   │   ├── CMakeSystem.cmake
    │   │   │   ├── CompilerIdC
    │   │   │   │   ├── a.out
    │   │   │   │   └── CMakeCCompilerId.c
    │   │   │   ├── CompilerIdCUDA
    │   │   │   │   ├── a.out
    │   │   │   │   ├── CMakeCUDACompilerId.cu
    │   │   │   │   └── tmp
    │   │   │   │       ├── a_dlink.fatbin
    │   │   │   │       ├── a_dlink.fatbin.c
    │   │   │   │       ├── a_dlink.o
    │   │   │   │       ├── a_dlink.reg.c
    │   │   │   │       ├── a_dlink.sm_30.cubin
    │   │   │   │       ├── CMakeCUDACompilerId.cpp1.ii
    │   │   │   │       ├── CMakeCUDACompilerId.cpp4.ii
    │   │   │   │       ├── CMakeCUDACompilerId.cudafe1.c
    │   │   │   │       ├── CMakeCUDACompilerId.cudafe1.cpp
    │   │   │   │       ├── CMakeCUDACompilerId.cudafe1.gpu
    │   │   │   │       ├── CMakeCUDACompilerId.cudafe1.stub.c
    │   │   │   │       ├── CMakeCUDACompilerId.fatbin
    │   │   │   │       ├── CMakeCUDACompilerId.fatbin.c
    │   │   │   │       ├── CMakeCUDACompilerId.module_id
    │   │   │   │       ├── CMakeCUDACompilerId.o
    │   │   │   │       ├── CMakeCUDACompilerId.ptx
    │   │   │   │       └── CMakeCUDACompilerId.sm_30.cubin
    │   │   │   └── CompilerIdCXX
    │   │   │       ├── a.out
    │   │   │       └── CMakeCXXCompilerId.cpp
    │   │   ├── cmake.check_cache
    │   │   ├── CMakeDirectoryInformation.cmake
    │   │   ├── CMakeOutput.log
    │   │   ├── feature_tests.bin
    │   │   ├── feature_tests.c
    │   │   ├── feature_tests.cxx
    │   │   ├── Makefile2
    │   │   ├── Makefile.cmake
    │   │   ├── myplugins.dir
    │   │   │   ├── build.make
    │   │   │   ├── cmake_clean.cmake
    │   │   │   ├── cmake_device_link.o
    │   │   │   ├── CUDA.includecache
    │   │   │   ├── DependInfo.cmake
    │   │   │   ├── depend.internal
    │   │   │   ├── depend.make
    │   │   │   ├── dlink.txt
    │   │   │   ├── flags.make
    │   │   │   ├── link.txt
    │   │   │   ├── plugin
    │   │   │   │   └── yololayer.cu.o
    │   │   │   └── progress.make
    │   │   ├── progress.marks
    │   │   ├── TargetDirectories.txt
    │   │   ├── yolov5_cls.dir
    │   │   │   ├── build.make
    │   │   │   ├── cmake_clean.cmake
    │   │   │   ├── cmake_device_link.o
    │   │   │   ├── CUDA.includecache
    │   │   │   ├── CXX.includecache
    │   │   │   ├── DependInfo.cmake
    │   │   │   ├── depend.internal
    │   │   │   ├── depend.make
    │   │   │   ├── dlink.txt
    │   │   │   ├── flags.make
    │   │   │   ├── link.txt
    │   │   │   ├── progress.make
    │   │   │   ├── src
    │   │   │   │   ├── calibrator.cpp.o
    │   │   │   │   ├── model.cpp.o
    │   │   │   │   ├── postprocess.cpp.o
    │   │   │   │   └── preprocess.cu.o
    │   │   │   └── yolov5_cls.cpp.o
    │   │   ├── yolov5_det.dir
    │   │   │   ├── build.make
    │   │   │   ├── cmake_clean.cmake
    │   │   │   ├── cmake_device_link.o
    │   │   │   ├── CUDA.includecache
    │   │   │   ├── CXX.includecache
    │   │   │   ├── DependInfo.cmake
    │   │   │   ├── depend.internal
    │   │   │   ├── depend.make
    │   │   │   ├── dlink.txt
    │   │   │   ├── flags.make
    │   │   │   ├── link.txt
    │   │   │   ├── progress.make
    │   │   │   ├── src
    │   │   │   │   ├── calibrator.cpp.o
    │   │   │   │   ├── model.cpp.o
    │   │   │   │   ├── postprocess.cpp.o
    │   │   │   │   └── preprocess.cu.o
    │   │   │   └── yolov5_det.cpp.o
    │   │   └── yolov5_seg.dir
    │   │       ├── build.make
    │   │       ├── cmake_clean.cmake
    │   │       ├── cmake_device_link.o
    │   │       ├── CUDA.includecache
    │   │       ├── CXX.includecache
    │   │       ├── DependInfo.cmake
    │   │       ├── depend.internal
    │   │       ├── depend.make
    │   │       ├── dlink.txt
    │   │       ├── flags.make
    │   │       ├── link.txt
    │   │       ├── progress.make
    │   │       ├── src
    │   │       │   ├── calibrator.cpp.o
    │   │       │   ├── model.cpp.o
    │   │       │   ├── postprocess.cpp.o
    │   │       │   └── preprocess.cu.o
    │   │       └── yolov5_seg.cpp.o
    │   ├── cmake_install.cmake
    │   ├── libmyplugins.so
    │   ├── Makefile
    │   ├── yolov5_cls
    │   ├── yolov5_det
    │   └── yolov5_seg
    ├── CMakeLists.txt
    ├── gen_wts.py
    ├── images -> ../yolov3-spp/samples
    ├── plugin
    │   ├── yololayer.cu
    │   └── yololayer.h
    ├── predictor.py
    ├── __pycache__
    │   ├── predictor.cpython-36.pyc
    │   └── yolov5_det_trt.cpython-36.pyc
    ├── README.md
    ├── src
    │   ├── calibrator.cpp
    │   ├── calibrator.h
    │   ├── config.h
    │   ├── cuda_utils.h
    │   ├── logging.h
    │   ├── macros.h
    │   ├── model.cpp
    │   ├── model.h
    │   ├── postprocess.cpp
    │   ├── postprocess.h
    │   ├── preprocess.cu
    │   ├── preprocess.h
    │   ├── types.h
    │   └── utils.h
    ├── yolov5_cls.cpp
    ├── yolov5_cls_trt.py
    ├── yolov5_det.cpp
    ├── yolov5_det_cuda_python.py
    ├── yolov5_det_trt.py
    ├── yolov5_seg.cpp
    └── yolov5_seg_trt.py

36 directories, 212 files
