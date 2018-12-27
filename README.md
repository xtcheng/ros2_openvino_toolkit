# Introduction

The OpenVINO™ (Open visual inference and neural network optimization) toolkit provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this runtime framework extends  workloads across Intel® hardware (including accelerators) and maximizes performance.
* Enables CNN-based deep learning inference at the edge
* Supports heterogeneous execution across computer vision accelerators—CPU, GPU, Intel® Movidius™ Neural Compute Stick, and FPGA—using a common API
* Speeds up time to market via a library of functions and preoptimized kernels
* Includes optimized calls for OpenCV and OpenVX*

## Design Architecture
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

## Logic Flow
From the view of logic implementation, the package introduces the definitions of parameter manager, pipeline and pipeline manager. The below picture depicts how these entities co-work together when the corresponding program is launched.

![Logic_Flow](https://github.com/LewisLiuPub/ros2_openvino_toolkit/blob/guide-info/doc/impletation_logic.PNG "OpenVINO RunTime Logic Flow")

Once a corresponding program is launched with a specified .yaml config file passed in the .launch.py file or via commandline, _**parameter manager**_ analyzes the configurations about pipeline and the whole framework, then shares the parsed configuration information with pipeline procedure. A _**pipeline instance**_ is created by following the configuration info and is added into _**pipeline manager**_ for lifecycle control and inference action triggering.

The contents in **.yaml config file** should be well structured and follow the supported rules and entity names. Please see [the configuration guidance]() for how to create or edit the config files.

**Pipeline** fulfills the whole data handling process: initiliazing Input Component for image data gathering and formating; building up the structured inference network and passing the formatted data through the inference network; transfering the inference results and handling output, etc.

**Pipeline manager** manages all the created pipelines according to the inference requests or external demands (say, system exception, resource limitation, or end user's operation). Because of co-working with resource management and being aware of the whole framework, it covers the ability of performance optimization by sharing system resource between pipelines and reducing the burden of data copy.

# Supported Features
## Diversal Input Components
Currently, the package support several kinds of input resources of gaining image data:

|Input Resource|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|
|Image Topic| any ROS topic which is structured in image message.|
|Image File| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video File| Any video file which can be parsed by openCV.|

## Inference Implementations
Currently, the inference feature list is supported:

|Inference|Description|
|-----------------------|------------------------------------------------------------------|
|Face Detection|Object Detection task applied to face recognition using a sequence of neural networks.|
|Emotion Recognition| Emotion recognition based on detected face image.|
|Age & Gender Recognition| Age and gener recognition based on detected face image.|
|Head Pose Estimation| Head pose estimation based on detected face image.|
|Object Detection| object detection based on SSD-based trained models.|
|Vehicle Detection| Vehicle and passenger detection based on Intel models.|
|Object Segmentation| object detection and segmentation.|

## ROS interfaces and outputs
### Topic
- Face Detection:
```/openvino_toolkit/faces```([object_msgs:msg:ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Emotion Detection:
```/openvino_toolkit/emotions```([people_msgs:msg:EmotionsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/EmotionsStamped.msg))
- Age and Gender Detection:
```/openvino_toolkit/age_genders```([people_msgs:msg:AgeGenderStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/AgeGenderStamped.msg))
- Head Pose:
```/openvino_toolkit/headposes```([people_msgs:msg:HeadPoseStamped](https://github.com/intel/ros2_openvino_toolkit/blob/master/people_msgs/msg/HeadPoseStamped.msg))

### Service
TBD

### RViz
RViz dispaly is also supported by the composited topic of original image frame with inference result.
To show in RViz tool, add an image marker with the composited topic:
_To be added_

### Image Window
OpenCV based image window is natively supported by the package.
To enable window, Image Window output should be added into the output choices in .yaml config file. see [the config file guidance](TBD) for checking/adding this feature in your launching.

## Demo Result Snapshots
See below pictures for the demo result snapshots.

# Installation & Launching
## Dependencies Installation
One-step installation scripts are provided for the dependencies' installation. Please see [the guide]() for details.
Note that beside the opensource version of the openVINO toolkit, Intel also releases the toolkit in Binary version. If you prefer to use the binary version, you may follow [this guide]().

## Install ROS2 OpenVINO Toolkit
### Download
```bash
mkdir -p ~/ros2_overlay_ws/src
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_openvino_toolkit
git clone https://github.com/intel/ros2_object_msgs
git clone https://github.com/ros-perception/vision_opencv -b ros2
```
### Build
```
source ~/ros2_ws/install/local_setup.bash
cd ~/ros2_overlay_ws
colcon build --symlink-install
```

## Launch ROS2 OpenVINO Toolkit
1.  Preparation
	-  copy label files (excute _once_)
	```bash
	sudo cp ~/ros2_overlay_ws/src/ros2_openvino/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
	```
	- set OpenVINO toolkit ENV
	```bash
	source /opt/intel/computer_vision_sdk/bin/setupvars.sh
	```
	- set ENV LD_LIBRARY_PATH
	```bash
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
	```

2.  run sample code with command-line arguments for <b>image or video</b> detection
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m <model_path_for_face_detection> -m_em <model_path_for_emotion_detection> -m_ag <model_path_for_age_and_gender_detection> -m-hp <model_path_for_headpose_detection> -i <input_type> -i_path <file_directory> -d <device> -d_em <device> -d_ag <device> -d_hp <device>
```

Options for &lt;input_type&gt;: _Image_ or _Video_.

Options for &lt;device&gt;: _CPU_ or _GPU_. Default is _CPU_. 
    
For example,
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.xml -m_em /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml -m_ag /opt/intel/computer_vision_sdk/deployment_tools/intel_models/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml -m-hp /opt/intel/computer_vision_sdk/deployment_tools/intel_models/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml -i Image -i_path <file_directory> -d CPU -d_em CPU -d_ag GPU -d_hp GPU
```

3. run sample code with command-line arguments for <b>video stream from camera</b> detection
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m <model_path_for_face_detection> -m_em <model_path_for_emotion_detection> -m_ag <model_path_for_age_and_gender_detection> -m-hp <model_path_for_headpose_detection> -i <input_type> -d <device> -d_em <device> -d_ag <device> -d_hp <device>
```

Options for &lt;input_type&gt;: _StandardCamera_ or _RealSenseCamera_. Default is _StandardCamera_.

Options for &lt;device&gt;: _CPU_ or _GPU_. Default is _CPU_. 
    
For example, 
```bash
ros2 run dynamic_vino_sample dynamic_vino_sample -m /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32/face-detection-adas-0001.xml -m_em /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32/emotions-recognition-retail-0003.xml -m_ag /opt/intel/computer_vision_sdk/deployment_tools/intel_models/age-gender-recognition-retail-0013/FP32/age-gender-recognition-retail-0013.xml -m-hp /opt/intel/computer_vision_sdk/deployment_tools/intel_models/head-pose-estimation-adas-0001/FP32/head-pose-estimation-adas-0001.xml -i StandardCamera -d CPU -d_em CPU -d_ag GPU -d_hp GPU
```
4. run sample code with parameters extracted from [yaml](https://github.com/intel/ros2_openvino_toolkit/blob/master/vino_param_lib/param/pipeline.yaml).
```bash
ros2 run dynamic_vino_sample pipeline_with_params
```

# TODO Features

# More Information

