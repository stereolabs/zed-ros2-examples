#include "topic_benchmark_component.hpp"

#include <rcutils/logging_macros.h>
#include <rclcpp/time.hpp>
#include <chrono>

namespace stereolabs
{
TopicBenchmarkComponent::TopicBenchmarkComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("topic_benchmark", options), mSubQos(1)
{
  mTopicAvailable.store(false);
  init();
}

TopicBenchmarkComponent::~TopicBenchmarkComponent()
{
  if (mTopicTimer)
    mTopicTimer->cancel();
}

void TopicBenchmarkComponent::init()
{
  getParameters();

  if (!mTopicAvailable.load())
  {
    mTopicTimer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(500),
                                    std::bind(&TopicBenchmarkComponent::updateTopicInfo, this));
  }
}

template <typename T>
void TopicBenchmarkComponent::getParam(std::string paramName, T defValue, T& outVal, std::string log_info)
{
  try
  {
    declare_parameter(paramName, rclcpp::ParameterValue(defValue));
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& ex)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal))
  {
    RCLCPP_WARN_STREAM(get_logger(), "The parameter '"
                                         << paramName << "' is not available or is not valid, using the default value: "
                                         << defValue);
  }

  if (!log_info.empty())
  {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void TopicBenchmarkComponent::getParameters()
{
  RCLCPP_INFO(get_logger(), "***** Benchmark parameters *****");

  getParam("topic_name", DEFAULT_TOPIC_NAME, mTopicName, "Topic name: ");
  if (mTopicName == DEFAULT_TOPIC_NAME)
  {
    RCLCPP_WARN(get_logger(), "Please remap the parameter 'topic_name' with the name of the parameter to benchmark.");
  }
  getParam("avg_win_size", mWinSize, mWinSize, "Average window size: ");
}

void TopicBenchmarkComponent::updateTopicInfo()
{
  mTopicAvailable.store(false);

  std::map<std::string, std::vector<std::string>> topic_infos = this->get_topic_names_and_types();
  for (const auto& topic_it : topic_infos)
  {
    std::string topic_name = topic_it.first;
    mTopicTypes = topic_it.second;

    if (topic_name == mTopicName)
    {
      // iterate over all topic types
      for (const auto& topic_type : mTopicTypes)
      {
        mTopicAvailable.store(true);
        RCLCPP_INFO_STREAM(get_logger(), "Found topic: " << mTopicName << "of type: ");
      }
    }
  }

  if (!mTopicAvailable.load())
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for topic '" << mTopicName << "' to be published.");
  else
  {
    if (mTopicTimer)
      mTopicTimer->cancel();
  }
}

}  // namespace stereolabs

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::TopicBenchmarkComponent)