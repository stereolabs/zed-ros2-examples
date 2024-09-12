#include "yolo_detector.hpp"
#include "NvOnnxParser.h"

#include "logging.hpp"

#include <filesystem>
#include <sensor_msgs/image_encodings.hpp>

using namespace nvinfer1;
namespace fs = std::filesystem;
namespace stereolabs
{

static Logger gLogger;
ZedYoloDetector::ZedYoloDetector(const rclcpp::NodeOptions & options)
: ZedCustomOd(options)
{
  RCLCPP_INFO(get_logger(), "***********************************");
  RCLCPP_INFO(get_logger(), " ZED Yolo detector Component ");
  RCLCPP_INFO(get_logger(), "************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "***********************************");

  // Read the parameters
  readParams();

  // ----> Create debug publisher
  if (_publishDebugImg) {
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.qos_overriding_options =
      rclcpp::QosOverridingOptions::with_default_policies();
    std::string pub_topic_name = std::string("~/") + "debug_image";
    _pubDebugImg = create_publisher<
      sensor_msgs::msg::Image>(
      pub_topic_name, rclcpp::QoS(10), pub_opt);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Advertised on topic: " << _pubDebugImg->get_topic_name());
  }
  // <---- Create debug publisher

  // Build the engine if required
  if (_buildEngine) {
    OptimDim dyn_dim_profile;
    dyn_dim_profile.setFromImgSize(_imgProcSize);
    if (!build_engine(_onnxFullPath, _engineFullPath, dyn_dim_profile) ) {
      exit(EXIT_FAILURE);
    }
  }

  // Initialize the custom detector
  init();

  RCLCPP_INFO_STREAM(
    get_logger(), "*** Node " << std::string(
      get_fully_qualified_name()) << " ready ***");
}

ZedYoloDetector::~ZedYoloDetector()
{
  if (_initialized) {
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(d_input));
    CUDA_CHECK(cudaFree(d_output));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();

    delete[] h_input;
    delete[] h_output;
  }
  _initialized = false;
}

void ZedYoloDetector::readParams()
{
  readGeneralParams();
  readEngineParams();
}

void ZedYoloDetector::readGeneralParams()
{
  RCLCPP_INFO(get_logger(), "*** YOLO Detector General Parameters ***");

  getParam(
    "general.conf_thresh", _confThresh, _confThresh,
    " * Confidence thresh.: ", true);
  getParam(
    "general.nms_thresh", _nmsThresh, _nmsThresh,
    " * NMS thresh.: ", true);
}

void ZedYoloDetector::readEngineParams()
{
  RCLCPP_INFO(get_logger(), "*** YOLO Detector Engine Parameters ***");

  getParam("yolo_engine.build_engine", _buildEngine, _buildEngine);
  RCLCPP_INFO_STREAM(get_logger(), " * Build engine: " << (_buildEngine ? "TRUE" : "FALSE"));
  getParam(
    "yolo_engine.onnx_model", _onnxFullPath, _onnxFullPath, " * ONNX model: ",
    false);
  _onnxFullPath = expand_home_dir(_onnxFullPath);
  getParam(
    "yolo_engine.engine_path", _enginePath, _enginePath, " * Engine path: ",
    false);
  _enginePath = expand_home_dir(_enginePath);
  getParam(
    "yolo_engine.engine_name", _engineName, _engineName, " * Engine name: ",
    false);

  // ----> Create Engine Full Path
  _engineFullPath = _enginePath;
  if (_engineFullPath.back() != '/') {
    _engineFullPath += "/";
  }
  _engineFullPath += _engineName;
  RCLCPP_INFO_STREAM(get_logger(), " + Engine full path: " << _engineFullPath);
  // <---- Create Engine Full Path

  ZedCustomOd::getParam(
    "yolo_engine.img_size", _imgProcSize, _imgProcSize,
    " * Image size: ", false);
}

bool ZedYoloDetector::build_engine(
  std::string onnx_path, std::string engine_path,
  OptimDim dyn_dim_profile)
{
  RCLCPP_INFO(
    get_logger(),
    "*** Creating the Inference Engine from the ONNX model... ");

  RCLCPP_INFO(get_logger(), " * File path check");
  // ----> Create the engine path if not existing
  if (!fs::exists(_enginePath)) {
    if (!fs::create_directories(_enginePath)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error creating the Engine folder:" << _enginePath);
      exit(EXIT_FAILURE);
    }
  }
  // <---- Create the engine path if not existing

  // ----> Verify the presence of an engine file with the same name
  if (fs::exists(_engineFullPath)) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "The Engine file '" << _engineFullPath << "' already exists!");
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Please manually delete it for generating a new engine or set the parameter 'yolo_engine.build_engine' to false to use the existing.");
    exit(EXIT_FAILURE);
  }
  // <---- Verify the presence of an engine file with the same name

  RCLCPP_INFO(get_logger(), " * Read ONNX File");
  std::vector<uint8_t> onnx_file_content;
  if (readFile(onnx_path, onnx_file_content)) {
    RCLCPP_ERROR_STREAM(get_logger(), " Cannot read the ONNX model file: " << onnx_path);
    return false;
  }

  if ((!onnx_file_content.empty())) {

    RCLCPP_INFO(get_logger(), " * Create Infer Builder");
    ICudaEngine * engine;
    // Create engine (onnx)
    gLogger.setReportableSeverity(Severity::kINFO);
    auto builder = nvinfer1::createInferBuilder(gLogger);
    if (!builder) {
      RCLCPP_ERROR_STREAM(get_logger(), "nvinfer1::createInferBuilder failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), " * Create Network");
    auto explicitBatch = 1U <<
      static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);

    if (!network) {
      RCLCPP_ERROR_STREAM(get_logger(), "createNetworkV2 failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), " * Create Builder Config");
    auto config = builder->createBuilderConfig();
    if (!config) {
      RCLCPP_ERROR_STREAM(get_logger(), "createBuilderConfig failed");
      return false;
    }

    ////////// Dynamic dimensions handling : support only 1 size at a time
    if (!dyn_dim_profile.tensor_name.empty()) {

      RCLCPP_INFO_STREAM(
        get_logger(), " * Create Optimization Profile: '" <<
          dyn_dim_profile.tensor_name << ":" <<
          dyn_dim_profile.size.d[0] << "x" <<
          dyn_dim_profile.size.d[1] << "x" <<
          dyn_dim_profile.size.d[2] << "x" <<
          dyn_dim_profile.size.d[3]
      );

      IOptimizationProfile * profile = builder->createOptimizationProfile();

      profile->setDimensions(
        dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kMIN, dyn_dim_profile.size);
      profile->setDimensions(
        dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kOPT, dyn_dim_profile.size);
      profile->setDimensions(
        dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kMAX, dyn_dim_profile.size);

      config->addOptimizationProfile(profile);
      builder->setMaxBatchSize(1);
    }

    RCLCPP_INFO(get_logger(), " * Create Parser");
    auto parser = nvonnxparser::createParser(*network, gLogger);
    if (!parser) {
      RCLCPP_ERROR_STREAM(get_logger(), "nvonnxparser::createParser failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), " * Parse ONNX file");
    bool parsed = false;
    unsigned char * onnx_model_buffer = onnx_file_content.data();
    size_t onnx_model_buffer_size = onnx_file_content.size() * sizeof(char);
    parsed = parser->parse(onnx_model_buffer, onnx_model_buffer_size);

    if (!parsed) {
      RCLCPP_ERROR_STREAM(get_logger(), "ONNX file parsing failed");
      return false;
    }

    if (builder->platformHasFastFp16()) {
      RCLCPP_INFO_STREAM(get_logger(), "FP16 enabled!");
      config->setFlag(BuilderFlag::kFP16);
    }

    //////////////// Actual engine building

    RCLCPP_INFO(get_logger(), " * Build Engine. THIS STEP WILL TAKE A WHILE. Please wait...");
    engine = builder->buildEngineWithConfig(*network, *config);

    onnx_file_content.clear();

    // write plan file if it is specified
    if (engine == nullptr) {
      RCLCPP_ERROR_STREAM(get_logger(), "buildEngineWithConfig failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), " * Serialize Engine");
    IHostMemory * ptr = engine->serialize();
    assert(ptr);
    if (ptr == nullptr) {return 1;}

    FILE * fp = fopen(engine_path.c_str(), "wb");
    fwrite(reinterpret_cast<const char *>(ptr->data()), ptr->size() * sizeof(char), 1, fp);
    fclose(fp);

    parser->destroy();
    network->destroy();
    config->destroy();
    builder->destroy();

    engine->destroy();

    RCLCPP_INFO_STREAM(get_logger(), "... Inference Engine ready: " << _engineFullPath);
    return true;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "ONNX file not valid");
    return false;
  }
}

void ZedYoloDetector::init()
{
  RCLCPP_INFO(get_logger(), "*** Node initialization ***");

  // ----> Verify the presence of an engine file with the same name
  if (!fs::exists(_engineFullPath)) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "The Engine file '" << _engineFullPath << "' does not exist!");
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Please set the parameter 'yolo_engine.build_engine' to true to generate a new engine model, or verify the correctness of the parameters 'yolo_engine.engine_path' and 'yolo_engine.engine_name' ");
    exit(EXIT_FAILURE);
  }
  // <---- Verify the presence of an engine file with the same name

  // deserialize the .engine and run inference
  std::ifstream file(_engineFullPath, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR_STREAM(get_logger(), "read " << _engineFullPath << " error!");
    exit(EXIT_FAILURE);
  }

  char * trtModelStream = nullptr;
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  trtModelStream = new char[size];
  if (!trtModelStream) {
    RCLCPP_ERROR(get_logger(), "Failed creating the trtModelStream variable");
    exit(EXIT_FAILURE);
  }
  file.read(trtModelStream, size);
  file.close();

  // prepare input data ---------------------------
  runtime = createInferRuntime(gLogger);
  if (runtime == nullptr) {
    RCLCPP_ERROR(get_logger(), "'createInferRuntime' failed");
    exit(EXIT_FAILURE);
  }
  engine = runtime->deserializeCudaEngine(trtModelStream, size);
  if (engine == nullptr) {
    RCLCPP_ERROR(get_logger(), "'deserializeCudaEngine' failed");
    exit(EXIT_FAILURE);
  }
  context = engine->createExecutionContext();
  if (context == nullptr) {
    RCLCPP_ERROR(get_logger(), "'createExecutionContext' failed");
    exit(EXIT_FAILURE);
  }

  delete[] trtModelStream;
  const int bindings = engine->getNbBindings();
  if (bindings != 2) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Wrong 'getNbBindings' value. Got " << engine->getNbBindings() << " expected 2");
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < bindings; i++) {
    if (engine->bindingIsInput(i)) {
      input_binding_name = engine->getBindingName(i);
      Dims bind_dim = engine->getBindingDimensions(i);
      _imgProcSize = bind_dim.d[3];
      //input_height = bind_dim.d[2];
      inputIndex = i;
      RCLCPP_INFO_STREAM(get_logger(), "Inference size : " << _imgProcSize << "x" << _imgProcSize);
    }    //if (engine->getTensorIOMode(engine->getBindingName(i)) == TensorIOMode::kOUTPUT)
    else {
      output_name = engine->getBindingName(i);
      // fill size, allocation must be done externally
      outputIndex = i;
      Dims bind_dim = engine->getBindingDimensions(i);
      size_t batch = bind_dim.d[0];
      if (batch > batch_size) {
        RCLCPP_ERROR_STREAM(get_logger(), "batch > 1 not supported");
        exit(EXIT_FAILURE);
      }
      size_t dim1 = bind_dim.d[1];
      size_t dim2 = bind_dim.d[2];

      if (dim1 > dim2) {
        // Yolov6 1x8400x85 //  85=5+80=cxcy+cwch+obj_conf+cls_conf
        out_dim = dim1;
        out_box_struct_number = 5;
        out_class_number = dim2 - out_box_struct_number;
        yolo_model_version = YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV6;
        RCLCPP_INFO_STREAM(get_logger(), "YOLOV6 format");
      } else {
        // Yolov8 1x84x8400
        out_dim = dim2;
        out_box_struct_number = 4;
        out_class_number = dim1 - out_box_struct_number;
        yolo_model_version = YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV8_V5;
        RCLCPP_INFO_STREAM(get_logger(), "YOLOV8/YOLOV5 format");
      }
    }
  }
  output_size = out_dim * (out_class_number + out_box_struct_number);
  h_input = new float[batch_size * 3 * _imgProcSize * _imgProcSize];
  h_output = new float[batch_size * output_size];
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(
    cudaMalloc(
      reinterpret_cast<void **>(&d_input),
      batch_size * 3 * _imgProcSize * _imgProcSize * sizeof(float)));
  CUDA_CHECK(
    cudaMalloc(
      reinterpret_cast<void **>(&d_output),
      batch_size * output_size * sizeof(float)));
  // Create stream
  CUDA_CHECK(cudaStreamCreate(&stream));

  if (batch_size != 1) {
    RCLCPP_ERROR_STREAM(get_logger(), "batch > 1 not supported");
    exit(EXIT_FAILURE);                      // This sample only support batch 1 for now
  }
  _initialized = true;
}

void ZedYoloDetector::doInference()
{
  RCLCPP_INFO_STREAM(get_logger(), "ZedYoloDetector::doInference()");

  if (_zedImg.empty()) {
    RCLCPP_WARN(get_logger(), "ZedYoloDetector::doInference() -> Received an empty image");
    return;
  }

  // ----> Preparing inference
  _inferenceResult.clear(); // Clear the previous result
  int img_w = _zedImg.cols;
  int img_h = _zedImg.rows;
  size_t frame_s = _imgProcSize * _imgProcSize;

  // letterbox BGR to RGB
  cv::Mat pr_img = preprocess_img(_zedImg, _imgProcSize, _imgProcSize);

  int idx = 0;
  int batch = 0;
  for (int row = 0; row < _imgProcSize; ++row) {
    uchar * uc_pixel = pr_img.data + row * pr_img.step;
    for (int col = 0; col < _imgProcSize; ++col) {
      h_input[batch * 3 * frame_s + idx] = (float)uc_pixel[2] / 255.0f;
      h_input[batch * 3 * frame_s + idx + frame_s] = (float)uc_pixel[1] / 255.0f;
      h_input[batch * 3 * frame_s + idx + 2 * frame_s] =
        (float)uc_pixel[0] / 255.0f;
      uc_pixel += 3;
      ++idx;
    }
  }
  // <---- Preparing inference

  // ----> Inference
  // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
  CUDA_CHECK(
    cudaMemcpyAsync(
      d_input, h_input, batch_size * 3 * frame_s * sizeof(float),
      cudaMemcpyHostToDevice, stream));

  std::vector<void *> d_buffers_nvinfer(2);
  d_buffers_nvinfer[inputIndex] = d_input;
  d_buffers_nvinfer[outputIndex] = d_output;
  context->enqueueV2(&d_buffers_nvinfer[0], stream, nullptr);

  CUDA_CHECK(
    cudaMemcpyAsync(
      h_output, d_output, batch_size * output_size * sizeof(float),
      cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
  // <---- Inference

  // ----> Extraction
  float scalingFactor =
    std::min(static_cast<float>(_imgProcSize) / img_w, static_cast<float>(_imgProcSize) / img_h);
  float xOffset = (_imgProcSize - scalingFactor * img_w) * 0.5f;
  float yOffset = (_imgProcSize - scalingFactor * img_h) * 0.5f;
  scalingFactor = 1.f / scalingFactor;
  float scalingFactor_x = scalingFactor;
  float scalingFactor_y = scalingFactor;

  switch (yolo_model_version) {
    default:
    case YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV8_V5:
      {
        // https://github.com/triple-Mu/YOLOv8-TensorRT/blob/df11cec3abaab7fefb28fb760f1cebbddd5ec826/csrc/detect/normal/include/yolov8.hpp#L343
        auto num_channels = out_class_number + out_box_struct_number;
        auto num_anchors = out_dim;
        auto num_labels = out_class_number;

        auto & dw = xOffset;
        auto & dh = yOffset;

        auto & width = img_w;
        auto & height = img_h;

        cv::Mat output = cv::Mat(
          num_channels,
          num_anchors,
          CV_32F,
          static_cast<float *>(h_output)
        );
        output = output.t();
        for (int i = 0; i < num_anchors; i++) {
          auto row_ptr = output.row(i).ptr<float>();
          auto bboxes_ptr = row_ptr;
          auto scores_ptr = row_ptr + out_box_struct_number;
          auto max_s_ptr = std::max_element(scores_ptr, scores_ptr + num_labels);
          float score = *max_s_ptr;
          if (score > _confThresh) {
            int label = max_s_ptr - scores_ptr;

            BBoxInfo bbi;

            float x = *bboxes_ptr++ - dw;
            float y = *bboxes_ptr++ - dh;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;

            float x0 = clamp((x - 0.5f * w) * scalingFactor_x, 0.f, width);
            float y0 = clamp((y - 0.5f * h) * scalingFactor_y, 0.f, height);
            float x1 = clamp((x + 0.5f * w) * scalingFactor_x, 0.f, width);
            float y1 = clamp((y + 0.5f * h) * scalingFactor_y, 0.f, height);

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            bbi.box.x1 = x0;
            bbi.box.y1 = y0;
            bbi.box.x2 = x1;
            bbi.box.y2 = y1;

            if ((bbi.box.x1 > bbi.box.x2) || (bbi.box.y1 > bbi.box.y2)) {break;}

            bbi.label = label;
            bbi.prob = score;

            _inferenceResult.push_back(bbi);
          }
        }
        break;
      }
    case YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV6:
      {
        // https://github.com/DefTruth/lite.ai.toolkit/blob/1267584d5dae6269978e17ffd5ec29da496e503e/lite/ort/cv/yolov6.cpp#L97

        auto & dw_ = xOffset;
        auto & dh_ = yOffset;

        auto & width = img_w;
        auto & height = img_h;

        const unsigned int num_anchors = out_dim;     // n = ?
        const unsigned int num_classes = out_class_number;     // - out_box_struct_number; // 80

        for (unsigned int i = 0; i < num_anchors; ++i) {
          const float * offset_obj_cls_ptr = h_output + (i * (num_classes + 5));      // row ptr
          float obj_conf = offset_obj_cls_ptr[4];       /*always == 1 for some reasons*/
          float cls_conf = offset_obj_cls_ptr[5];

          // The confidence is remapped because it output basically garbage with conf < ~0.1 and we don't want to clamp it either
          const float conf_offset = 0.1;
          const float input_start = 0;
          const float output_start = input_start;
          const float output_end = 1;
          const float input_end = output_end - conf_offset;

          float conf = (obj_conf * cls_conf) - conf_offset;
          if (conf < 0) {conf = 0;}
          conf = (conf - input_start) / (input_end - input_start) * (output_end - output_start) +
            output_start;

          if (conf > _confThresh) {

            unsigned int label = 0;
            for (unsigned int j = 0; j < num_classes; ++j) {
              float tmp_conf = offset_obj_cls_ptr[j + 5];
              if (tmp_conf > cls_conf) {
                cls_conf = tmp_conf;
                label = j;
              }
            }         // argmax

            BBoxInfo bbi;

            float cx = offset_obj_cls_ptr[0];
            float cy = offset_obj_cls_ptr[1];
            float w = offset_obj_cls_ptr[2];
            float h = offset_obj_cls_ptr[3];
            float x1 = ((cx - w / 2.f) - (float) dw_) * scalingFactor_x;
            float y1 = ((cy - h / 2.f) - (float) dh_) * scalingFactor_y;
            float x2 = ((cx + w / 2.f) - (float) dw_) * scalingFactor_x;
            float y2 = ((cy + h / 2.f) - (float) dh_) * scalingFactor_y;

            bbi.box.x1 = std::max(0.f, x1);
            bbi.box.y1 = std::max(0.f, y1);
            bbi.box.x2 = std::min(x2, (float) width - 1.f);
            bbi.box.y2 = std::min(y2, (float) height - 1.f);

            if ((bbi.box.x1 > bbi.box.x2) || (bbi.box.y1 > bbi.box.y2)) {break;}

            bbi.label = label;
            bbi.prob = conf;

            _inferenceResult.push_back(bbi);
          }
        }
        break;
      }
  }
  // <---- Extraction

  // NMS
  applyNonMaximumSuppression();

  // BBoxInfo to Detection2D
  generateRosMsg();

  RCLCPP_INFO_STREAM(get_logger(), " * Detected " << _inferenceResult.size() << " objects.");
}

void ZedYoloDetector::generateRosMsg()
{
  _detections.clear();

  cv::Mat resImg;
  _zedImg.copyTo(resImg);

  for (const auto & det : _inferenceResult) {
    vision_msgs::msg::Detection2D rosDet;

    rosDet.id = ""; // No unique ID at this level

    // Note: results must be published with normalized values to be image size agnostic
    rosDet.bbox.size_x = static_cast<double>(fabs(det.box.x2 - det.box.x1)) / _zedImg.cols;
    rosDet.bbox.size_y = static_cast<double>(fabs(det.box.y2 - det.box.y1)) / _zedImg.rows;
    rosDet.bbox.center.position.x = (static_cast<double>(det.box.x1) + (0.5 * rosDet.bbox.size_x)) /
      _zedImg.cols;
    rosDet.bbox.center.position.y = (static_cast<double>(det.box.y1) + (0.5 * rosDet.bbox.size_y)) /
      _zedImg.rows;

    vision_msgs::msg::ObjectHypothesisWithPose obj;
    obj.hypothesis.class_id = std::to_string(det.label);
    obj.hypothesis.score = det.prob;

    rosDet.results.push_back(obj);

    _detections.push_back(rosDet);

    if (_pubDebugImg) {
      cv::Rect r;
      r.x = det.box.x1;
      r.y = det.box.y1;
      r.width = det.box.x2 - det.box.x1;
      r.height = det.box.y2 - det.box.y1;
      cv::rectangle(resImg, r, cv::Scalar(220, 180, 20), 2);
    }
  }

  if (_pubDebugImg) {
    std::unique_ptr<sensor_msgs::msg::Image> msg = std::make_unique<sensor_msgs::msg::Image>();

    msg->header.frame_id = _detFrameId;
    msg->header.stamp = this->get_clock()->now();
    msg->encoding = sensor_msgs::image_encodings::BGR8;
    msg->width = resImg.cols;
    msg->height = resImg.rows;
    msg->step = resImg.step;
    int num = 1;  // for endianness detection
    msg->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    size_t size = msg->step * msg->height;
    msg->data.resize(size);
    memcpy(reinterpret_cast<char *>((&msg->data[0])), &resImg.data[0], size);

    _pubDebugImg->publish(std::move(msg));
  }
}

#define WEIGHTED_NMS

void ZedYoloDetector::applyNonMaximumSuppression()
{
  auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
      if (x1min > x2min) {
        std::swap(x1min, x2min);
        std::swap(x1max, x2max);
      }
      return x1max < x2min ? 0 : std::min(x1max, x2max) - x2min;
    };

  auto computeIoU = [&overlap1D](BBox & bbox1, BBox & bbox2) -> float {
      float overlapX = overlap1D(bbox1.x1, bbox1.x2, bbox2.x1, bbox2.x2);
      float overlapY = overlap1D(bbox1.y1, bbox1.y2, bbox2.y1, bbox2.y2);
      float area1 = (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1);
      float area2 = (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1);
      float overlap2D = overlapX * overlapY;
      float u = area1 + area2 - overlap2D;
      return u == 0 ? 0 : overlap2D / u;
    };

  std::stable_sort(
    _inferenceResult.begin(), _inferenceResult.end(), [](const BBoxInfo & b1, const BBoxInfo & b2) {
      return b1.prob > b2.prob;
    });

  std::vector<BBoxInfo> out;

#if defined(WEIGHTED_NMS)
  std::vector<std::vector<BBoxInfo>> weigthed_nms_candidates;
#endif
  for (auto & i : _inferenceResult) {
    bool keep = true;

#if defined(WEIGHTED_NMS)
    int j_index = 0;
#endif

    for (auto & j : out) {
      if (keep) {
        float overlap = computeIoU(i.box, j.box);
        keep = overlap <= _nmsThresh;
#if defined(WEIGHTED_NMS)
        if (!keep && fabs(j.prob - i.prob) < 0.52f) {       // add label similarity check
          weigthed_nms_candidates[j_index].push_back(i);
        }
#endif
      } else {
        break;
      }

#if defined(WEIGHTED_NMS)
      j_index++;
#endif

    }
    if (keep) {
      out.push_back(i);
#if defined(WEIGHTED_NMS)
      weigthed_nms_candidates.emplace_back();
      weigthed_nms_candidates.back().clear();
#endif
    }
  }

#if defined(WEIGHTED_NMS)

  for (int i = 0; i < out.size(); i++) {
    // the best confidence
    BBoxInfo & best = out[i];
    float sum_tl_x = best.box.x1 * best.prob;
    float sum_tl_y = best.box.y1 * best.prob;
    float sum_br_x = best.box.x2 * best.prob;
    float sum_br_y = best.box.y2 * best.prob;

    float weight = best.prob;
    for (auto & it : weigthed_nms_candidates[i]) {
      sum_tl_x += it.box.x1 * it.prob;
      sum_tl_y += it.box.y1 * it.prob;
      sum_br_x += it.box.x2 * it.prob;
      sum_br_y += it.box.y2 * it.prob;
      weight += it.prob;
    }

    weight = 1.f / weight;
    best.box.x1 = sum_tl_x * weight;
    best.box.y1 = sum_tl_y * weight;
    best.box.x2 = sum_br_x * weight;
    best.box.y2 = sum_br_y * weight;
  }

#endif

  _inferenceResult = out;
}

} // namespace stereolabs

// *************************************************************************
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedYoloDetector)
// *************************************************************************
