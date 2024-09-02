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

namespace stereolabs
{

class ZedYoloDetector : public ZedCustomOd
{
public:
  ZED_CUSTOM_OD_COMPONENT_PUBLIC
  ZedYoloDetector(const rclcpp::NodeOptions & options);

  virtual ~ZedYoloDetector() {}

protected:
  virtual void init() override;
  virtual void doInference() override;
};

} // namespace stereolabs

#endif // YOLO_DETECTOR_HPP_
