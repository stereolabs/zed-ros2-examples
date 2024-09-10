#include "yolo_detector.hpp"
#include "NvOnnxParser.h"

#include "logging.hpp"

#include <filesystem>

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

  // Build the engine if required
  if (_buildEngine) {
    OptimDim dyn_dim_profile;
    dyn_dim_profile.setFromImgSize(_imgSize);
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

void ZedYoloDetector::readParams()
{
  readEngineParams();
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

  ZedCustomOd::getParam("yolo_engine.img_size", _imgSize, _imgSize, " * Image size: ", false);
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
    RCLCPP_ERROR_STREAM(get_logger(), "batch > 1 not supported");
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
