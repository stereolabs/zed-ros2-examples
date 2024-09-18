// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YOLO_DETECTOR_HPP_
#define YOLO_DETECTOR_HPP_

#include "custom_od_visibility_control.hpp"
#include "custom_od_component_base.hpp"

#include "utils.hpp"

#include <NvInfer.h>

namespace stereolabs
{
enum class YOLO_MODEL_VERSION_OUTPUT_STYLE
{
  YOLOV6,
  YOLOV8_V5
};

typedef struct _bbox
{
  float x1;
  float y1;
  float x2;
  float y2;
} BBox;

typedef struct _bboxInfo
{
  BBox box;
  int label;
  float prob;
} BBoxInfo;

typedef struct _optimDim
{
  nvinfer1::Dims4 size;
  std::string tensor_name;

  bool setFromImgSize(int img_size)
  {
    size.nbDims = 4;
    // assuming batch is 1 and channel is 3
    size.d[0] = 1;
    size.d[1] = 3;
    size.d[2] = img_size;
    size.d[3] = img_size;

    tensor_name = "images";
    return true;
  }
} OptimDim;

/*!
 * @brief Implement a YOLO Detector nodethat subscribes to ZED Left image
 *        topic and returns a vector of 2D detections
 */
class ZedYoloDetector : public ZedCustomOd
{
public:
  ZED_CUSTOM_OD_COMPONENT_PUBLIC
  explicit ZedYoloDetector(const rclcpp::NodeOptions & options);

  virtual ~ZedYoloDetector();

protected:
  virtual void init() override;
  virtual void doInference() override;

  void readParams();
  void readGeneralParams();
  void readEngineParams();

private:
  bool build_engine(std::string onnx_path, std::string engine_path, OptimDim dyn_dim_profile);
  void applyNonMaximumSuppression();
  void generateRosMsg();

private:
  // ----> Parameters
  bool _buildEngine = false;
  std::string _onnxFullPath;
  std::string _enginePath;
  std::string _engineName;
  int _imgProcSize = 512;
  float _nmsThresh = 0.4f;
  float _confThresh = 0.3f;
  std::string _yoloModelVersion;
  bool _publishDebugImg = false;
  // <---- Parameters

  std::string _engineFullPath;

  // Get input dimension size
  std::string input_binding_name = "images";   // input
  std::string output_name = "classes";
  int inputIndex, outputIndex;
  size_t batch_size = 1;
  // Yolov6 1x8400x85 //  85=5+80=cxcy+cwch+obj_conf+cls_conf //https://github.com/DefTruth/lite.ai.toolkit/blob/1267584d5dae6269978e17ffd5ec29da496e503e/lite/ort/cv/yolov6.cpp#L97
  // Yolov8/yolov5 1x84x8400
  size_t out_dim = 8400;
  size_t out_class_number = 80; /*for COCO*/
  size_t out_box_struct_number = 4;   // https://github.com/ultralytics/yolov3/issues/750#issuecomment-569783354
  size_t output_size = 0;

  YOLO_MODEL_VERSION_OUTPUT_STYLE yolo_model_version;

  float * h_input;
  float * h_output;
  float * d_input;
  float * d_output;

  nvinfer1::IRuntime * runtime;
  nvinfer1::ICudaEngine * engine;
  nvinfer1::IExecutionContext * context;
  cudaStream_t stream;

  bool _initialized = false;
  std::vector<BBoxInfo> _inferenceResult;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pubDebugImg;
};

} // namespace stereolabs

#endif // YOLO_DETECTOR_HPP_
