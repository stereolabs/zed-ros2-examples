// Copyright 2023 Stereolabs
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

#include "zed_aruco_localization_component.hpp"

#include <sstream>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/image_encodings.hpp>

#include "aruco.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)

namespace stereolabs
{

ZedArucoLocComponent::ZedArucoLocComponent(const rclcpp::NodeOptions & options)
: Node("zed_aruco_loc_node", options), _defaultQoS(1), _detRunning(false)
{
  RCLCPP_INFO(get_logger(), "*********************************");
  RCLCPP_INFO(get_logger(), " ZED ArUco Localization Component ");
  RCLCPP_INFO(get_logger(), "*********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "*********************************");

  /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
     * and durability set as "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use compatible
     * parameters.
     */

  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

  _defaultQoS.keep_last(10);
  _defaultQoS.reliable();
  _defaultQoS.durability_volatile();

  // Initialize detection time for throttling
  _detTime = get_clock()->now();

  // Load parameters
  getParams();

  // ----> TF2 Transform
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tfListener =
    std::make_unique<tf2_ros::TransformListener>(*_tfBuffer);  // Start TF Listener thread
  _tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // <---- TF2 Transform

  // Create image publisher
  _pubDetect = image_transport::create_camera_publisher(
    this, "out/aruco_result",
    _defaultQoS.get_rmw_qos_profile());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubDetect.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << _pubDetect.getInfoTopic());

  // Create camera image subscriber
  _subImage = image_transport::create_camera_subscription(
    this, "in/zed_image", std::bind(&ZedArucoLocComponent::camera_callback, this, _1, _2), "raw",
    _defaultQoS.get_rmw_qos_profile());

  RCLCPP_INFO_STREAM(get_logger(), "Subscribed on topic: " << _subImage.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Subscribed on topic: " << _subImage.getInfoTopic());
}

template<typename T>
void ZedArucoLocComponent::getParam(
  std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The parameter '" <<
        paramName <<
        "' is not available or is not valid, using the default value: " <<
        defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void ZedArucoLocComponent::getParams()
{
  getGeneralParams();
  getMarkerParams();
}

void ZedArucoLocComponent::getGeneralParams()
{
  RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

  getParam("general.marker_count", _markerCount, _markerCount);
  RCLCPP_INFO(get_logger(), " * Marker count: ");
  getParam("general.marker_size", _markerSize, _markerSize);
  RCLCPP_INFO(get_logger(), " * Marker size [m]: ");
  getParam("general.detection_rate", _detRate, _detRate);
  RCLCPP_INFO(get_logger(), " * Detection rate [Hz]: ");
  getParam("general.world_frame_id", _worldFrameId, _worldFrameId);
  RCLCPP_INFO(get_logger(), " * World frame id: ");
}

void ZedArucoLocComponent::getMarkerParams()
{
  RCLCPP_INFO(get_logger(), "*** MARKER parameters ***");

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  for (int i = 0; i < _markerCount; i++) {
    ArucoPose aruco_pose;
    std::stringstream ns;
    ns << "marker_" << std::setfill('0') << std::setw(3) << i;

    std::string par_pref = ns.str() + ".";

    RCLCPP_INFO_STREAM(get_logger(), " * " << par_pref);

    std::string par_name = par_pref + "aruco_id";
    getParam(par_name, aruco_pose.idx, aruco_pose.idx, "   * ArUco idx: ");

    std::stringstream ss;
    ss << "marker_" << std::setfill('0') << std::setw(3) << aruco_pose.idx;
    aruco_pose.marker_frame_id = ss.str();

    par_name = par_pref + "position";
    declare_parameter(par_name, rclcpp::ParameterValue(aruco_pose.position), read_only_descriptor);
    if (!get_parameter(par_name, aruco_pose.position)) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "The parameter '" <<
          par_name <<
          "' is not available or is not valid.");
      exit(EXIT_FAILURE);
    }
    if (aruco_pose.position.size() != 3) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "The parameter '" << par_name <<
          "' must be a vector of 3 values of type FLOAT64 (double).");
      exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(
      get_logger(), "   * Position: [%g,%g,%g]", aruco_pose.position[0], aruco_pose.position[1],
      aruco_pose.position[2]);

    par_name = par_pref + "orientation";
    declare_parameter(
      par_name, rclcpp::ParameterValue(aruco_pose.orientation),
      read_only_descriptor);
    if (!get_parameter(par_name, aruco_pose.orientation)) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "The parameter '" <<
          par_name <<
          "' is not available or is not valid.");
      exit(EXIT_FAILURE);
    }
    if (aruco_pose.orientation.size() != 3) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "The parameter '" << par_name <<
          "' must be a vector of 3 values of type FLOAT64 (double).");
      exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(
      get_logger(), "   * Orientation: [%g,%g,%g]", aruco_pose.orientation[0],
      aruco_pose.orientation[1], aruco_pose.orientation[2]);

    _tagPoses[aruco_pose.idx] = aruco_pose;
  }
}

void ZedArucoLocComponent::camera_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  // Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(get_logger(), "The input topic image requires 'BGRA8' encoding");
    exit(EXIT_FAILURE);
  }

  if (_detRunning) {
    return;
  }

  double det_elapsed_sec = (get_clock()->now() - _detTime).nanoseconds() / 1e9;

  if (det_elapsed_sec < (1.0 / _detRate)) {
    return;
  }

  _detRunning = true;

  // Init TF2
  void initTFs();

  // Time statistics
  rclcpp::Time start;
  double elapsed_sec;


  // Is result image subscribed?
  bool res_sub = (_pubDetect.getNumSubscribers() > 0);

  // ----> Check for correct input image encoding
  if (img->encoding != sensor_msgs::image_encodings::BGRA8) {
    RCLCPP_ERROR(get_logger(), "The input topic image requires 'BGRA8' encoding");
    _detRunning = false;
    return;
  }
  // <---- Check for correct input image encoding

  RCLCPP_INFO_STREAM(get_logger(), "*****************************");
  RCLCPP_INFO_STREAM(get_logger(), "    ArUco detection");

  // ----> Convert BGRA image for processing by using OpenCV
  start = get_clock()->now();
  void * data = const_cast<void *>(reinterpret_cast<const void *>(&img->data[0]));
  cv::Mat bgra(img->height, img->width, CV_8UC4, data);
  cv::Mat bgr, gray; // bgr is used to publish the detection image, gray for ArUco processing

  cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
  cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  RCLCPP_INFO_STREAM(get_logger(), " * Color conversion: " << elapsed_sec << " sec");
  // ----> Convert BGRA image for processing by using OpenCV

  // ----> Detect ArUco Markers
  start = get_clock()->now();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  cv::aruco::detectMarkers(bgr, dictionary, corners, ids);
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  RCLCPP_INFO_STREAM(get_logger(), " * Marker detection: " << elapsed_sec << " sec");
  // <---- Detect ArUco Markers

  if (corners.empty()) {
    _detRunning = false;
    RCLCPP_INFO_STREAM(get_logger(), "  No Markers in view");
    RCLCPP_INFO_STREAM(get_logger(), "*****************************");
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), " * Detected tags: " << ids.size());

  // ----> Refine the result
  start = get_clock()->now();
  for (size_t i = 0; i < corners.size(); ++i) {
    cv::cornerSubPix(
      gray, corners[i], cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
  }
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  if (corners.empty()) {
    _detRunning = false;
    RCLCPP_INFO_STREAM(get_logger(), "  No Markers in view");
    RCLCPP_INFO_STREAM(get_logger(), "*****************************");
    return;
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Subpixel refinement: " << elapsed_sec << " sec");
  // <---- Refine the result

  // ----> Estimate Marker positions
  start = get_clock()->now();
  std::vector<cv::Vec3d> rvecs, tvecs;

  cv::Matx33d camera_matrix = cv::Matx33d::eye(); // Camera matrix
  camera_matrix(0, 0) = cam_info->k[0];
  camera_matrix(1, 1) = cam_info->k[4];
  camera_matrix(0, 2) = cam_info->k[2];
  camera_matrix(1, 2) = cam_info->k[5];
  cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros(); // No distortions if subscribing to rectified ZED images

  cv::aruco::estimatePoseSingleMarkers(
    corners, _markerSize, camera_matrix, dist_coeffs, rvecs,
    tvecs);
  elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
  RCLCPP_INFO_STREAM(get_logger(), " * Marker poses estimation: " << elapsed_sec << " sec");
  // <---- Estimate Marker positions

  // ----> Find closest marker
  size_t nearest_aruco_index = 999;
  double nearest_distance = 1e9;
  for (size_t i = 0; i < ids.size(); i++) {
    double x = tvecs[i](0);
    double y = tvecs[i](1);
    double z = tvecs[i](2);
    double current_distance = sqrt(x * x + y * y + z * z);
    if (nearest_distance > current_distance) {
      nearest_distance = current_distance;
      nearest_aruco_index = i;
    }
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Nearest marker: " << ids[nearest_aruco_index] << " -> " << nearest_distance << "m");
  // <---- Find closest marker

  // Update Marker TFs
  publishMarkerTFs();

  // ----> Calculate new camera pose
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(
    cv::Vec3d(
      {rvecs[nearest_aruco_index](0), rvecs[nearest_aruco_index](1),
        rvecs[nearest_aruco_index](2)}), rot);
  // Convert to a tf2::Matrix3x3
  tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

  // Create a transform and convert to a Pose
  tf2::Transform tf2_transform(tf2_rot, tf2::Vector3(
      tvecs[nearest_aruco_index](0), tvecs[nearest_aruco_index](1),
      tvecs[nearest_aruco_index](2)));

  RCLCPP_INFO(
    get_logger(), "tf2_transform: [%.3f,%.3f,%.3f] [%.3f,%.3f,%.3f,%.3f]",
    tf2_transform.getOrigin().getX(), tf2_transform.getOrigin().getY(),
    tf2_transform.getOrigin().getZ(),
    tf2_transform.getRotation().getX(),
    tf2_transform.getRotation().getY(),
    tf2_transform.getRotation().getZ(), tf2_transform.getRotation().getW());

  // Convert from optical frame to camera base frame
  tf2::Transform base_pose = _opt2base.inverse() * tf2_transform.inverse() * _opt2base;

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = get_clock()->now();

  transformStamped.header.frame_id = _tagPoses[ids[nearest_aruco_index]].marker_frame_id;
  transformStamped.child_frame_id = "zed_aruco";

  transformStamped.transform.rotation.x = base_pose.getRotation().getX();
  transformStamped.transform.rotation.y = base_pose.getRotation().getY();
  transformStamped.transform.rotation.z = base_pose.getRotation().getZ();
  transformStamped.transform.rotation.w = base_pose.getRotation().getW();

  transformStamped.transform.translation.x = base_pose.getOrigin().getX();
  transformStamped.transform.translation.y = base_pose.getOrigin().getY();
  transformStamped.transform.translation.z = base_pose.getOrigin().getZ();

  _tfBroadcaster->sendTransform(transformStamped);
  // <---- Calculate new camera pose

  // ----> Draw and publish the results
  if (res_sub) {
    start = get_clock()->now();
    cv::aruco::drawDetectedMarkers(bgr, corners, ids);

    // Create the output message and copy coverted data
    std::shared_ptr<sensor_msgs::msg::Image> out_bgr = std::make_shared<sensor_msgs::msg::Image>();

    out_bgr->header.stamp = img->header.stamp;
    out_bgr->header.frame_id = img->header.frame_id;
    out_bgr->height = bgr.rows;
    out_bgr->width = bgr.cols;

    int num = 1; // for endianness detection
    out_bgr->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    out_bgr->step = bgr.step;

    size_t size = out_bgr->step * out_bgr->height;
    out_bgr->data.resize(size);

    out_bgr->encoding = sensor_msgs::image_encodings::BGR8;
    memcpy(reinterpret_cast<char *>((&out_bgr->data[0])), &bgr.data[0], size);

    // Publish the new image message coupled with camera info from the original message
    _pubDetect.publish(out_bgr, cam_info);
    elapsed_sec = (get_clock()->now() - start).nanoseconds() / 1e9;
    RCLCPP_INFO_STREAM(get_logger(), " * Publish image result: " << elapsed_sec << " sec");
  }
  // <---- Draw and publish the results

  // TODO 3. `set_pose` service call

  _detTime = get_clock()->now();

  RCLCPP_INFO_STREAM(get_logger(), "*****************************");
  _detRunning = false;
}

void ZedArucoLocComponent::publishMarkerTFs()
{
  for (auto pose:_tagPoses) {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = get_clock()->now();

    transformStamped.header.frame_id = _worldFrameId;
    transformStamped.child_frame_id = pose.second.marker_frame_id;

    tf2::Quaternion q;
    q.setRPY(pose.second.orientation[0], pose.second.orientation[1], pose.second.orientation[2]);

    transformStamped.transform.rotation.x = q.getX();
    transformStamped.transform.rotation.y = q.getY();
    transformStamped.transform.rotation.z = q.getZ();
    transformStamped.transform.rotation.w = q.getW();

    transformStamped.transform.translation.x = pose.second.position[0];
    transformStamped.transform.translation.y = pose.second.position[1];
    transformStamped.transform.translation.z = pose.second.position[2];

    _tfBroadcaster->sendTransform(transformStamped);
  }
}

bool ZedArucoLocComponent::getTransformFromTf(
  std::string targetFrame, std::string sourceFrame,
  tf2::Transform & out_tr)
{
  std::string msg;
  geometry_msgs::msg::TransformStamped tf_ros;

  try {
    // ----> Without this a warning is returned the first time... why???
    _tfBuffer->canTransform(targetFrame, sourceFrame, TIMEZERO_ROS, 500ms, &msg);
    RCLCPP_INFO_STREAM(
      get_logger(),
      "canTransform '" << targetFrame.c_str() << "' -> '" << sourceFrame.c_str() << "':" <<
        msg.c_str());
    std::this_thread::sleep_for(3ms);
    // <---- Without this a warning is returned the first time... why???

    tf_ros = _tfBuffer->lookupTransform(targetFrame, sourceFrame, TIMEZERO_ROS, 1s);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not transform '%s' to '%s': %s",
      targetFrame.c_str(), sourceFrame.c_str(), ex.what());
    return false;
  }

  tf2::Vector3 origin;
  origin.setX(tf_ros.transform.translation.x);
  origin.setY(tf_ros.transform.translation.y);
  origin.setZ(tf_ros.transform.translation.z);
  out_tr.setOrigin(origin);

  tf2::Quaternion q;
  q.setX(tf_ros.transform.rotation.x);
  q.setY(tf_ros.transform.rotation.y);
  q.setZ(tf_ros.transform.rotation.z);
  q.setW(tf_ros.transform.rotation.w);
  out_tr.setRotation(q);

  return true;
}

void ZedArucoLocComponent::initTFs()
{
  // Publish the TF poses of the markers from the configuration file
  publishMarkerTFs();

  // Get the transform from camera optical frame to camera base frame
  std::string cam_opt_frame = "zed_left_camera_optical_frame"; // TODO(Walter) create from params
  std::string cam_base_frame = "zed_camera_link"; // TODO(Walter) create from params
  bool tf_ok = getTransformFromTf(cam_opt_frame, cam_base_frame, _opt2base);
  if (!tf_ok) {
    RCLCPP_ERROR(
      get_logger(),
      "The transform '%s' -> '%s' is not available. Please verify the parameters and the status of the 'ZED State Publisher' node.");
    exit(EXIT_FAILURE);
  }
}


}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedArucoLocComponent)
