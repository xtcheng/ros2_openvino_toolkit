# Introduction

The OpenVINO™ (Open visual inference and neural network optimization) toolkit provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this runtime framework extends  workloads across Intel® hardware (including accelerators) and maximizes performance.
* Enables CNN-based deep learning inference at the edge
* Supports heterogeneous execution across computer vision accelerators—CPU, GPU, Intel® Movidius™ Neural Compute Stick, and FPGA—using a common API
* Speeds up time to market via a library of functions and preoptimized kernels
* Includes optimized calls for OpenCV and OpenVX*

## Designing Architecture
From the view of hirarchical architecture design, the package is divided into different functional components, as shown in below picture. 

![OpenVINO_Architecture](https://github.com/LewisLiuPub/ros2_openvino_toolkit/blob/guide-info/doc/design_arch.PNG "OpenVINO RunTime Architecture")

- **Intel® OpenVINO™ toolkit** is leveraged to provide deep learning basic implementation for data inference. is free software that helps developers and data scientists speed up computer vision workloads, streamline deep learning inference and deployments,
and enable easy, heterogeneous execution across Intel® platforms from edge to cloud. It helps to:
   - Increase deep learning workload performance up to 19x1 with computer vision accelerators from Intel.
   - Unleash convolutional neural network (CNN)-based deep learning inference using a common API.
   - Speed development using optimized OpenCV* and OpenVX* functions.
- **ROS2 OpenVINO Runtime Framework** is the main body of this repo. it provides key logic implementation for pipeline lifecycle management, resource management and ROS system adapter, which extends Intel OpenVINO toolkit and libraries. Furthermore, this runtime framework provides ways to ease launching, configuration and data analytics and re-use.
- **Diversal Input resources** are the data resources to be infered and analyzed with the OpenVINO framework.
- **ROS interfaces and outputs** currently include _Topic_ and _service_. Natively, RViz output and CV image window output are also supported by refactoring topic message and inferrence results.
- **Optimized Models** provides by Model Optimizer component of Intel® OpenVINO™ toolkit. Imports trained models from various frameworks (Caffe*, Tensorflow*, MxNet*, ONNX*, Kaldi*) and converts them to a unified intermediate representation file. It also optimizes topologies through node merging, horizontal fusion, eliminating batch normalization, and quantization.It also supports graph freeze and graph summarize along with dynamic input freezing.

## Logic Design

# Supported Features


## Demo Result Snapshots

# Installation & Launching

# TODO Features

# More Information


This project is a ROS2 wrapper for CV API of OpenVINO™, providing the following features:
* Support CPU and GPU platforms
* Support standard USB camera and Intel® RealSense™ camera
* Support Video or Image file as detection source
* Face detection
* Emotion recognition
* Age and gender recognition
* Head pose recognition
* Object detection
* Demo application to show above detection and recognitions

**Note**:We provide two ways to install the OpenVINO™ toolkit:<br>
Install from Binary version [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit, please refer to [BINARY_VERSION_README.md](https://github.com/intel/ros2_openvino_toolkit/blob/master/doc/BINARY_VERSION_README.md).<br>
Install from OpenVINO toolkit open source code, please refer to [OPEN_SOURCE_CODE_README.md](https://github.com/intel/ros2_openvino_toolkit/blob/master/doc/OPEN_SOURCE_CODE_README.md)
