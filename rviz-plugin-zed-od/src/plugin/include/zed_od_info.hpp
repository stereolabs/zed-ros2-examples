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

#ifndef ZED_OD_INFO_HPP_
#define ZED_OD_INFO_HPP_

#include <QObject>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>

#include "visibility_control.hpp"
#include "zed_body_parts.hpp"

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

}  // namespace displays
}  // namespace rviz_plugin_zed_od

#endif  // ZED_OD_INFO_HPP_
