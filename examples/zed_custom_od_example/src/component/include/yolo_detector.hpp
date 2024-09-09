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

struct BBox
{
  float x1, y1, x2, y2;
};

struct BBoxInfo
{
  BBox box;
  int label;
  float prob;
};

struct OptimDim
{
  nvinfer1::Dims4 size;
  std::string tensor_name;

  bool setFromString(const std::string & arg)
  {
    // "images:1x3x512x512"
    std::vector<std::string> v_ = split_str(arg, ":");
    if (v_.size() != 2) {return true;}

    std::string dims_str = v_.back();
    std::vector<std::string> v = split_str(dims_str, "x");

    size.nbDims = 4;
    // assuming batch is 1 and channel is 3
    size.d[0] = 1;
    size.d[1] = 3;

    if (v.size() == 2) {
      size.d[2] = stoi(v[0]);
      size.d[3] = stoi(v[1]);
    } else if (v.size() == 3) {
      size.d[2] = stoi(v[1]);
      size.d[3] = stoi(v[2]);
    } else if (v.size() == 4) {
      size.d[2] = stoi(v[2]);
      size.d[3] = stoi(v[3]);
    } else {return true;}

    if (size.d[2] != size.d[3]) {
      std::cerr << "Warning only squared input are currently supported" << std::endl;
    }

    tensor_name = v_.front();
    return false;
  }
};

/*!
 * @brief
 */
class ZedYoloDetector : public ZedCustomOd
{
public:
  ZED_CUSTOM_OD_COMPONENT_PUBLIC
  explicit ZedYoloDetector(const rclcpp::NodeOptions & options);

  virtual ~ZedYoloDetector() {}

protected:
  virtual void init() override;
  virtual void doInference() override;

  void readParams();

private:
  int build_engine(std::string onnx_path, std::string engine_path, OptimDim dyn_dim_profile);

private:
  // ----> Parameters
  std::string _onnxPath;
  std::string _enginePath;
  std::string _engineName;
  float _nms = 0.4;
  std::string _yoloModelVersion;
  // <---- Parameters

  // Get input dimension size
  std::string input_binding_name = "images";   // input
  std::string output_name = "classes";
  int inputIndex, outputIndex;
  size_t input_width = 0, input_height = 0, batch_size = 1;
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

  bool is_init = false;
};

} // namespace stereolabs

#endif // YOLO_DETECTOR_HPP_
