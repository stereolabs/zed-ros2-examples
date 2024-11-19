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

#ifndef YOLO_UTILS_HPP_
#define YOLO_UTILS_HPP_

#include <vector>
#include <string>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "yolo_detector.hpp"

std::vector<std::vector<int>> const COCO_CLASS_COLORS = {
  {0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48}, {77, 190, 238},
  {162, 20, 47}, {76, 76, 76}, {153, 153, 153}, {255, 0, 0}, {255, 128, 0}, {191, 191, 0},
  {0, 255, 0}, {0, 0, 255}, {170, 0, 255}, {85, 85, 0}, {85, 170, 0}, {85, 255, 0},
  {170, 85, 0}, {170, 170, 0}, {170, 255, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0},
  {0, 85, 128}, {0, 170, 128}, {0, 255, 128}, {85, 0, 128}, {85, 85, 128}, {85, 170, 128},
  {85, 255, 128}, {170, 0, 128}, {170, 85, 128}, {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
  {255, 85, 128}, {255, 170, 128}, {255, 255, 128}, {0, 85, 255}, {0, 170, 255}, {0, 255, 255},
  {85, 0, 255}, {85, 85, 255}, {85, 170, 255}, {85, 255, 255}, {170, 0, 255}, {170, 85, 255},
  {170, 170, 255}, {170, 255, 255}, {255, 0, 255}, {255, 85, 255}, {255, 170, 255}, {85, 0, 0},
  {128, 0, 0}, {170, 0, 0}, {212, 0, 0}, {255, 0, 0}, {0, 43, 0}, {0, 85, 0},
  {0, 128, 0}, {0, 170, 0}, {0, 212, 0}, {0, 255, 0}, {0, 0, 43}, {0, 0, 85},
  {0, 0, 128}, {0, 0, 170}, {0, 0, 212}, {0, 0, 255}, {0, 0, 0}, {36, 36, 36},
  {73, 73, 73}, {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
  {80, 183, 189}, {128, 128, 0}};

std::vector<std::string> const COCO_CLASS_NAMES = {
  "person",
  "bicycle",
  "car",
  "motorcycle",
  "airplane",
  "bus",
  "train",
  "truck",
  "boat",
  "traffic light",
  "fire hydrant",
  "stop sign",
  "parking meter",
  "bench",
  "bird",
  "cat",
  "dog",
  "horse",
  "sheep",
  "cow",
  "elephant",
  "bear",
  "zebra",
  "giraffe",
  "backpack",
  "umbrella",
  "handbag",
  "tie",
  "suitcase",
  "frisbee",
  "skis",
  "snowboard",
  "sports ball",
  "kite",
  "baseball bat",
  "baseball glove",
  "skateboard",
  "surfboard",
  "tennis racket",
  "bottle",
  "wine glass",
  "cup",
  "fork",
  "knife",
  "spoon",
  "bowl",
  "banana",
  "apple",
  "sandwich",
  "orange",
  "broccoli",
  "carrot",
  "hot dog",
  "pizza",
  "donut",
  "cake",
  "chair",
  "couch",
  "potted plant",
  "bed",
  "dining table",
  "toilet",
  "tv",
  "laptop",
  "mouse",
  "remote",
  "keyboard",
  "cell phone",
  "microwave",
  "oven",
  "toaster",
  "sink",
  "refrigerator",
  "book",
  "clock",
  "vase",
  "scissors",
  "teddy bear",
  "hair drier",
  "toothbrush"
};

static void draw_objects(
  cv::Mat const & image,
  cv::Mat & res,
  const std::vector<stereolabs::BBoxInfo> & objs)
{
  res = image.clone();
  for (auto const & obj : objs) {
    size_t const idx_color{obj.label % COCO_CLASS_COLORS.size()};
    cv::Scalar const color{cv::Scalar(
        COCO_CLASS_COLORS[idx_color][0U], COCO_CLASS_COLORS[idx_color][1U],
        COCO_CLASS_COLORS[idx_color][2U])};

    cv::Rect const rect{static_cast<int>(obj.box.x1),
      static_cast<int>(obj.box.y1),
      static_cast<int>(obj.box.x2 - obj.box.x1),
      static_cast<int>(obj.box.y2 - obj.box.y1)};
    cv::rectangle(res, rect, color, 2);

    std::stringstream label;
    label << COCO_CLASS_NAMES[obj.label] << " [" << std::setw(3) << obj.prob << "]";

    int baseLine{0};
    cv::Size const label_size{cv::getTextSize(
        label.str().c_str(), cv::FONT_HERSHEY_PLAIN, 0.5, 1, &baseLine)};

    int const x{rect.x};
    int const y{std::min(rect.y + 1, res.rows)};

    cv::rectangle(
      res, cv::Rect(
        x, y, label_size.width,
        label_size.height + baseLine), {0, 0, 255}, -1);
    cv::putText(
      res, label.str().c_str(), cv::Point(
        x,
        y + label_size.height), cv::FONT_HERSHEY_PLAIN, 0.5, {255, 255, 255},
      1);
  }
}

#endif // YOLO_UTILS_HPP_
