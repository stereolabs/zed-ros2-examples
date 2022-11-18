// Copyright 2022 Stereolabs
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

#ifndef ZED_OD_INFO_HPP_
#define ZED_OD_INFO_HPP_

#include <QObject>
#include <memory>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <string>
#include <utility>
#include <vector>
#include <zed_interfaces/msg/objects_stamped.hpp>

#include "visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace rviz_plugin_zed_od
{
namespace displays
{

typedef std::shared_ptr<rviz_rendering::Shape> shapePtr;
typedef std::shared_ptr<rviz_rendering::BillboardLine> linePtr;

class ZED_OD_PLUGIN_PUBLIC ZedOdInfo : public QObject
{
  Q_OBJECT

public:
  explicit ZedOdInfo(
    zed_interfaces::msg::Object & obj, Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node = NULL);
  virtual ~ZedOdInfo();

  void updateShowLabel(bool show);
  void updateAlpha(float alpha);
  void updateInfo(zed_interfaces::msg::Object & obj);
  void updateShowBBox(bool show);
  void updateShowSkeleton(bool show);
  void updateLinkSize(float newval);
  void updateJointRadius(float newval);
  void updateLabelScale(float newval);

protected:
  void calculateColor();

private:
  std::string mObjName;

  // shapePtr mBBox;
  std::shared_ptr<rviz_rendering::MovableText> mLabel;

  std::vector<shapePtr> mBBoxCorners;
  std::vector<linePtr> mBBoxLines;

  std::vector<shapePtr> mSkelJoints;
  std::vector<linePtr> mSkelLinks;

  float mAlpha = 0.75f;
  Ogre::ColourValue mColorBBox;
  Ogre::ColourValue mColorSkel;

  Ogre::SceneManager * mSceneManager = nullptr;
  Ogre::SceneNode * mParentNode = nullptr;
  Ogre::SceneNode * mSceneNode = nullptr;
  Ogre::SceneNode * mSkelSceneNode = nullptr;
  Ogre::SceneNode * mBBoxSceneNode = nullptr;
  Ogre::SceneNode * mPivotSceneNode = nullptr;

  bool mShowLabel = true;
  bool mShowBBox = true;
  bool mShowSkel = true;

  shapePtr mPivot;

  int16_t mLabelId;

  float mLabelScale = 2.5;
  float mJointRadius = 0.1;
  float mLinkSize = 0.05;

  const float mSkelScale = 0.4f;
  const int mSkelColOffset = 50;

  // Unique identifier for each object
  static uint64_t mObjIdx;
};

enum class BODY_PARTS_18 {
  NOSE = 0,
  NECK = 1,
  RIGHT_SHOULDER = 2,
  RIGHT_ELBOW = 3,
  RIGHT_WRIST = 4,
  LEFT_SHOULDER = 5,
  LEFT_ELBOW = 6,
  LEFT_WRIST = 7,
  RIGHT_HIP = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE = 10,
  LEFT_HIP = 11,
  LEFT_KNEE = 12,
  LEFT_ANKLE = 13,
  RIGHT_EYE = 14,
  LEFT_EYE = 15,
  RIGHT_EAR = 16,
  LEFT_EAR = 17,
  LAST = 18
};

enum class BODY_PARTS_34 {
  PELVIS = 0,
  NAVAL_SPINE = 1,
  CHEST_SPINE = 2,
  NECK = 3,
  LEFT_CLAVICLE = 4,
  LEFT_SHOULDER = 5,
  LEFT_ELBOW = 6,
  LEFT_WRIST = 7,
  LEFT_HAND = 8,
  LEFT_HANDTIP = 9,
  LEFT_THUMB = 10,
  RIGHT_CLAVICLE = 11,
  RIGHT_SHOULDER = 12,
  RIGHT_ELBOW = 13,
  RIGHT_WRIST = 14,
  RIGHT_HAND = 15,
  RIGHT_HANDTIP = 16,
  RIGHT_THUMB = 17,
  LEFT_HIP = 18,
  LEFT_KNEE = 19,
  LEFT_ANKLE = 20,
  LEFT_FOOT = 21,
  RIGHT_HIP = 22,
  RIGHT_KNEE = 23,
  RIGHT_ANKLE = 24,
  RIGHT_FOOT = 25,
  HEAD = 26,
  NOSE = 27,
  LEFT_EYE = 28,
  LEFT_EAR = 29,
  RIGHT_EYE = 30,
  RIGHT_EAR = 31,
  LEFT_HEEL = 32,
  RIGHT_HEEL = 33,
  LAST = 34
};

const std::vector<std::pair<BODY_PARTS_18, BODY_PARTS_18>> BODY_BONES_18{
  {BODY_PARTS_18::NOSE, BODY_PARTS_18::NECK},
  {BODY_PARTS_18::NECK, BODY_PARTS_18::RIGHT_SHOULDER},
  {BODY_PARTS_18::RIGHT_SHOULDER, BODY_PARTS_18::RIGHT_ELBOW},
  {BODY_PARTS_18::RIGHT_ELBOW, BODY_PARTS_18::RIGHT_WRIST},
  {BODY_PARTS_18::NECK, BODY_PARTS_18::LEFT_SHOULDER},
  {BODY_PARTS_18::LEFT_SHOULDER, BODY_PARTS_18::LEFT_ELBOW},
  {BODY_PARTS_18::LEFT_ELBOW, BODY_PARTS_18::LEFT_WRIST},
  {BODY_PARTS_18::RIGHT_SHOULDER, BODY_PARTS_18::RIGHT_HIP},
  {BODY_PARTS_18::RIGHT_HIP, BODY_PARTS_18::RIGHT_KNEE},
  {BODY_PARTS_18::RIGHT_KNEE, BODY_PARTS_18::RIGHT_ANKLE},
  {BODY_PARTS_18::LEFT_SHOULDER, BODY_PARTS_18::LEFT_HIP},
  {BODY_PARTS_18::LEFT_HIP, BODY_PARTS_18::LEFT_KNEE},
  {BODY_PARTS_18::LEFT_KNEE, BODY_PARTS_18::LEFT_ANKLE},
  {BODY_PARTS_18::RIGHT_SHOULDER, BODY_PARTS_18::LEFT_SHOULDER},
  {BODY_PARTS_18::RIGHT_HIP, BODY_PARTS_18::LEFT_HIP},
  {BODY_PARTS_18::NOSE, BODY_PARTS_18::RIGHT_EYE},
  {BODY_PARTS_18::RIGHT_EYE, BODY_PARTS_18::RIGHT_EAR},
  {BODY_PARTS_18::NOSE, BODY_PARTS_18::LEFT_EYE},
  {BODY_PARTS_18::LEFT_EYE, BODY_PARTS_18::LEFT_EAR}};

const std::vector<std::pair<BODY_PARTS_34, BODY_PARTS_34>> BODY_BONES_34{
  {BODY_PARTS_34::PELVIS, BODY_PARTS_34::NAVAL_SPINE},
  {BODY_PARTS_34::NAVAL_SPINE, BODY_PARTS_34::CHEST_SPINE},
  {BODY_PARTS_34::CHEST_SPINE, BODY_PARTS_34::LEFT_CLAVICLE},
  {BODY_PARTS_34::LEFT_CLAVICLE, BODY_PARTS_34::LEFT_SHOULDER},
  {BODY_PARTS_34::LEFT_SHOULDER, BODY_PARTS_34::LEFT_ELBOW},
  {BODY_PARTS_34::LEFT_ELBOW, BODY_PARTS_34::LEFT_WRIST},
  {BODY_PARTS_34::LEFT_WRIST, BODY_PARTS_34::LEFT_HAND},
  {BODY_PARTS_34::LEFT_HAND, BODY_PARTS_34::LEFT_HANDTIP},
  {BODY_PARTS_34::LEFT_WRIST, BODY_PARTS_34::LEFT_THUMB},
  {BODY_PARTS_34::CHEST_SPINE, BODY_PARTS_34::RIGHT_CLAVICLE},
  {BODY_PARTS_34::RIGHT_CLAVICLE, BODY_PARTS_34::RIGHT_SHOULDER},
  {BODY_PARTS_34::RIGHT_SHOULDER, BODY_PARTS_34::RIGHT_ELBOW},
  {BODY_PARTS_34::RIGHT_ELBOW, BODY_PARTS_34::RIGHT_WRIST},
  {BODY_PARTS_34::RIGHT_WRIST, BODY_PARTS_34::RIGHT_HAND},
  {BODY_PARTS_34::RIGHT_HAND, BODY_PARTS_34::RIGHT_HANDTIP},
  {BODY_PARTS_34::RIGHT_WRIST, BODY_PARTS_34::RIGHT_THUMB},
  {BODY_PARTS_34::PELVIS, BODY_PARTS_34::LEFT_HIP},
  {BODY_PARTS_34::LEFT_HIP, BODY_PARTS_34::LEFT_KNEE},
  {BODY_PARTS_34::LEFT_KNEE, BODY_PARTS_34::LEFT_ANKLE},
  {BODY_PARTS_34::LEFT_ANKLE, BODY_PARTS_34::LEFT_FOOT},
  {BODY_PARTS_34::PELVIS, BODY_PARTS_34::RIGHT_HIP},
  {BODY_PARTS_34::RIGHT_HIP, BODY_PARTS_34::RIGHT_KNEE},
  {BODY_PARTS_34::RIGHT_KNEE, BODY_PARTS_34::RIGHT_ANKLE},
  {BODY_PARTS_34::RIGHT_ANKLE, BODY_PARTS_34::RIGHT_FOOT},
  {BODY_PARTS_34::CHEST_SPINE, BODY_PARTS_34::NECK},
  {BODY_PARTS_34::NECK, BODY_PARTS_34::HEAD},
  {BODY_PARTS_34::HEAD, BODY_PARTS_34::NOSE},
  {BODY_PARTS_34::NOSE, BODY_PARTS_34::LEFT_EYE},
  {BODY_PARTS_34::LEFT_EYE, BODY_PARTS_34::LEFT_EAR},
  {BODY_PARTS_34::NOSE, BODY_PARTS_34::RIGHT_EYE},
  {BODY_PARTS_34::RIGHT_EYE, BODY_PARTS_34::RIGHT_EAR},
  {BODY_PARTS_34::LEFT_ANKLE, BODY_PARTS_34::LEFT_HEEL},
  {BODY_PARTS_34::RIGHT_ANKLE, BODY_PARTS_34::RIGHT_HEEL},
  {BODY_PARTS_34::LEFT_HEEL, BODY_PARTS_34::LEFT_FOOT},
  {BODY_PARTS_34::RIGHT_HEEL, BODY_PARTS_34::RIGHT_FOOT}};
}  // namespace displays
}  // namespace rviz_plugin_zed_od

#endif  // ZED_OD_INFO_HPP_
