# ZED Custom Object Detection Example

This example demonstrates how to use the Custom Object Detection feature of the ZED SDK with the ZED ROS 2 Wrapper.
In this example a new component is created which subscribes to ZED Left image topics, performs AI inference with a custom AI engine, then publishes back the bounding boxes to the ZED Node.

To minimize latency, the example shows how to leverace Composition and Intra Process Communication, two advanced features available in ROS 2.

## Custom YOLO DETECTOR example

To explain how to create your Custom Detector we added an example based on Ultralytics [YOLO](https://docs.ultralytics.com/).

The example is using a TensorRT optimized ONNX model. It is compatible with YOLOv8, YOLOv5 and YOLOv6. It can be used with the default model trained on COCO dataset (80 classes) provided by the framework maintainers.

A custom detector can be trained with the same architecture. These tutorials walk you through the workflow of training a custom detector :

* Yolov6 https://github.com/meituan/YOLOv6/blob/main/docs/Train_custom_data.md
* Yolov5 https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data

The example is expecting a TensorRT engine, optimized from an ONNX model. The ONNX model can be exported from Pytorch using the original YOLO code.

Next we explain how to generate it for YOLO v8.

### Install YOLO v8

YOLOv8 can be installed directly from pip using the following command:

```bash
python -m pip install ultralytics
```

### ONNX file export from YOLO model

We'll use the CLI for export https://docs.ultralytics.com/modes/export/

```bash
yolo export model=yolov8n.pt format=onnx simplify=True dynamic=False imgsz=608
```

For a custom model model the weight file can be changed:

```bash
yolo export model=yolov8l_custom_model.pt format=onnx simplify=True dynamic=False imgsz=512
```

Please refer to the corresponding documentation for more details https://github.com/ultralytics/ultralytics

### Generate a TensorRT Engine from the ONNX model file

TensorRT apply heavy optimisation by processing the network structure itself and benchmarking all the available implementation of each inference function to take the fastest. The result in the inference engine.

We created the ZED YOLO node such that it can automatically generate a TensorRT engine from an ONNX mode file:

1. Open the `zed_yolo.yaml` file
2. Set the parameter `yolo_engine.build_engine` to `true`
3. Set the parameter `yolo_engine.onnx_model` with the full path to the ONNX file that you generated before
4. Set the parameter `yolo_engine.engine_path` with the path of the folder that will contain the generated engine file
5. Set the parameter `yolo_engine.engine_name` with the name of the generated engine file
6. Set the parameter `yolo_engine.img_size` with the size of the input images. **Note**: only square input image are supported

When you start the node, the initialization process will automatically generate the TensorRT engine.

### Use an existing TensorRT Engine

The TensorRT generation process can take a few minutes so we usually want to generate it the first time then saving it for later reload. The ZED YOLO node can use a ready Engine file:

1. Open the `zed_yolo.yaml` file
2. Set the parameter `yolo_engine.build_engine` to `false` to disable the Engine generation process
3. Set the parameter `yolo_engine.engine_path` with the path of the folder that contains the previously generated engine file
4. Set the parameter `yolo_engine.engine_name` with the name of the previously generated engine file
5. Set the parameter `yolo_engine.img_size` with the size of the input images used while generating the engine file

Now you can start the node and load a ready TensorRT Inference Engine file.