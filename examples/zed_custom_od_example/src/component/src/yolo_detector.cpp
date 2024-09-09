#include "yolo_detector.hpp"
#include "NvOnnxParser.h"

#include "logging.hpp"

using namespace nvinfer1;
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

  // Initialize the custom detector
  init();
}

void ZedYoloDetector::readParams()
{
  RCLCPP_INFO(get_logger(), "*** YOLO Detector Parameters ***");


}

int ZedYoloDetector::build_engine(
  std::string onnx_path, std::string engine_path,
  OptimDim dyn_dim_profile)
{


  std::vector<uint8_t> onnx_file_content;
  if (readFile(onnx_path, onnx_file_content)) {return 1;}

  if ((!onnx_file_content.empty())) {

    ICudaEngine * engine;
    // Create engine (onnx)
    RCLCPP_INFO(get_logger(), "Creating engine from onnx model");

    gLogger.setReportableSeverity(Severity::kINFO);
    auto builder = nvinfer1::createInferBuilder(gLogger);
    if (!builder) {
      RCLCPP_ERROR_STREAM(get_logger(), "createInferBuilder failed");
      return 1;
    }

    auto explicitBatch = 1U <<
      static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);

    if (!network) {
      RCLCPP_ERROR_STREAM(get_logger(), "createNetwork failed");
      return 1;
    }

    auto config = builder->createBuilderConfig();
    if (!config) {
      RCLCPP_ERROR_STREAM(get_logger(), "createBuilderConfig failed");
      return 1;
    }

    ////////// Dynamic dimensions handling : support only 1 size at a time
    if (!dyn_dim_profile.tensor_name.empty()) {

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

    auto parser = nvonnxparser::createParser(*network, gLogger);
    if (!parser) {
      RCLCPP_ERROR_STREAM(get_logger(), "nvonnxparser::createParser failed");
      return 1;
    }

    bool parsed = false;
    unsigned char * onnx_model_buffer = onnx_file_content.data();
    size_t onnx_model_buffer_size = onnx_file_content.size() * sizeof(char);
    parsed = parser->parse(onnx_model_buffer, onnx_model_buffer_size);

    if (!parsed) {
      RCLCPP_ERROR_STREAM(get_logger(), "onnx file parsing failed");
      return 1;
    }

    if (builder->platformHasFastFp16()) {
      RCLCPP_INFO_STREAM(get_logger(), "FP16 enabled!");
      config->setFlag(BuilderFlag::kFP16);
    }

    //////////////// Actual engine building

    engine = builder->buildEngineWithConfig(*network, *config);

    onnx_file_content.clear();

    // write plan file if it is specified
    if (engine == nullptr) {return 1;}
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

    return 0;
  } else {return 1;}
}

void ZedYoloDetector::init()
{
  // deserialize the .engine and run inference
  std::ifstream file(_engineName, std::ios::binary);
  if (!file.good()) {
    RCLCPP_ERROR_STREAM(get_logger(), "read " << _engineName << " error!");
    exit(EXIT_FAILURE);
  }

  char * trtModelStream = nullptr;
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  trtModelStream = new char[size];
  if (!trtModelStream) {
    exit(EXIT_FAILURE);
  }
  file.read(trtModelStream, size);
  file.close();

  // prepare input data ---------------------------
  runtime = createInferRuntime(gLogger);
  if (runtime == nullptr) {
    exit(EXIT_FAILURE);
  }
  engine = runtime->deserializeCudaEngine(trtModelStream, size);
  if (engine == nullptr) {
    exit(EXIT_FAILURE);
  }
  context = engine->createExecutionContext();
  if (context == nullptr) {
    exit(EXIT_FAILURE);
  }

  delete[] trtModelStream;
  if (engine->getNbBindings() != 2) {
    exit(EXIT_FAILURE);
  }


  const int bindings = engine->getNbBindings();
  for (int i = 0; i < bindings; i++) {
    if (engine->bindingIsInput(i)) {
      input_binding_name = engine->getBindingName(i);
      Dims bind_dim = engine->getBindingDimensions(i);
      input_width = bind_dim.d[3];
      input_height = bind_dim.d[2];
      inputIndex = i;
      RCLCPP_INFO_STREAM(get_logger(), "Inference size : " << input_height << "x" << input_width);
    }    //if (engine->getTensorIOMode(engine->getBindingName(i)) == TensorIOMode::kOUTPUT)
    else {
      output_name = engine->getBindingName(i);
      // fill size, allocation must be done externally
      outputIndex = i;
      Dims bind_dim = engine->getBindingDimensions(i);
      size_t batch = bind_dim.d[0];
      if (batch > batch_size) {
        RCLCPP_INFO_STREAM(get_logger(), "batch > 1 not supported");
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
  h_input = new float[batch_size * 3 * input_height * input_width];
  h_output = new float[batch_size * output_size];
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(
    cudaMalloc(
      reinterpret_cast<void **>(&d_input),
      batch_size * 3 * input_height * input_width * sizeof(float)));
  CUDA_CHECK(
    cudaMalloc(
      reinterpret_cast<void **>(&d_output),
      batch_size * output_size * sizeof(float)));
  // Create stream
  CUDA_CHECK(cudaStreamCreate(&stream));

  if (batch_size != 1) {
    exit(EXIT_FAILURE);                      // This sample only support batch 1 for now
  }
  is_init = true;
}

void ZedYoloDetector::doInference()
{
  RCLCPP_INFO_STREAM(get_logger(), "ZedYoloDetector::doInference()");
}


} // namespace stereolabs

// *************************************************************************
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedYoloDetector)
// *************************************************************************
