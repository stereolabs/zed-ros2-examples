// Copyright 2025 Stereolabs
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

#include "zed_od_display.hpp"

#include <iostream>

namespace rviz_plugin_zed_od
{
namespace displays
{
ZedOdDisplay::ZedOdDisplay()
{
  mPropAlpha = new rviz_common::properties::FloatProperty(
    "Transparency", 0.25f,
    "Structures transparency level [0 -> NONE, 1 -> Invisible]", this,
    SLOT(updateAlpha()));
  mPropAlpha->setMin(0);
  mPropAlpha->setMax(1);

  mPropShowSkeleton = new rviz_common::properties::BoolProperty(
    "Show Skeletons", true,
    "Enable/Disable skeletons visualization (if available)", this,
    SLOT(updateShowSkeleton()));

  mPropShowLabel = new rviz_common::properties::BoolProperty(
    "Show Labels", true, "Enable/Disable ID + Label visualization", this,
    SLOT(updateShowLabel()));

  mPropShowBBox = new rviz_common::properties::BoolProperty(
    "Show Bounding Boxes", true,
    "Enable/Disable Bounding boxes visualization", this,
    SLOT(updateShowBBox()));

  mPropLinkSize = new rviz_common::properties::FloatProperty(
    "Link Size", 0.05f,
    "Line size of the bounding box edges and skeleton links", this,
    SLOT(updateLinkSize()));

  mPropLinkSize->setMin(0.01);
  mPropAlpha->setMax(1.0);

  mPropJointRadius = new rviz_common::properties::FloatProperty(
    "Joint Radius", 0.1f,
    "Radius of the bounding box corners and skeleton joints", this,
    SLOT(updateJointRadius()));

  mPropLinkSize->setMin(0.01);
  mPropLinkSize->setMax(1.0);

  mPropLabelScale = new rviz_common::properties::FloatProperty(
    "Label Scale", 5.f, "Scale of the label", this, SLOT(updateLabelScale()));

  mPropLabelScale->setMin(0.5);
  mPropLabelScale->setMax(15.0);
}

ZedOdDisplay::~ZedOdDisplay() {}

void ZedOdDisplay::onInitialize() {MFDClass::onInitialize();}

void ZedOdDisplay::reset() {MFDClass::reset();}

void ZedOdDisplay::onEnable() {MFDClass::onEnable();}

void ZedOdDisplay::onDisable() {MFDClass::onDisable();}

void ZedOdDisplay::invalidateObjs()
{
  for (auto & it : mObjUpdated) {
    it.second = false;
  }
}

void ZedOdDisplay::removeNotValidObjs()
{
  std::map<int16_t, bool>::iterator it;
  for (it = mObjUpdated.begin(); it != mObjUpdated.end(); it++) {
    int16_t id = it->first;
    if (it->second == false) {
      auto obj_it = mObjects.find(id);
      if (obj_it != mObjects.end()) {
        mObjects.erase(obj_it);
      }
    }
  }
}

void ZedOdDisplay::processMessage(
  zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(
      msg->header, position,
      orientation))
  {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  if (position.isNaN() || orientation.isNaN()) {
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->objects.size() == 0) {
    mObjUpdated.clear();
    mObjects.clear();
  } else {
    invalidateObjs();

    for (auto object : msg->objects) {
      createOrUpdateObject(object);
    }

    removeNotValidObjs();

    mPropShowSkeleton->setHidden(!msg->objects[0].skeleton_available);
  }
}

void ZedOdDisplay::createOrUpdateObject(zed_msgs::msg::Object & obj)
{
  int16_t id = obj.label_id;
  if (id == -1 && obj.tracking_available) {  // Not a valid ID?
    return;
  }
  if (obj.tracking_available && obj.tracking_state != 1) {  // Tracking not OK?
    return;
  }
  if (qIsNaN(obj.position[0]) || qIsNaN(obj.position[1]) ||
    qIsNaN(obj.position[2]))
  {
    return;
  }

  mObjUpdated[id] = true;

  auto it = mObjects.find(id);
  if (it == mObjects.end() || id == -1) {
    objectPtr newObj =
      std::make_shared<ZedOdInfo>(obj, scene_manager_, scene_node_);
    mObjects[id] = newObj;
    updateAlpha();
    updateShowLabel();
    updateShowBBox();
    updateShowSkeleton();
    updateLinkSize();
    updateJointRadius();
    updateLabelScale();
  } else {
    if (it->second) {
      it->second->updateInfo(obj);
    }
  }
}

void ZedOdDisplay::updateAlpha()
{
  float alpha = 1.0 - mPropAlpha->getFloat();

  for (auto const & obj : mObjects) {
    obj.second->updateAlpha(alpha);
  }
}

void ZedOdDisplay::updateShowSkeleton()
{
  bool show = mPropShowSkeleton->getBool();

  for (auto const & obj : mObjects) {
    obj.second->updateShowSkeleton(show);
  }
}

void ZedOdDisplay::updateShowLabel()
{
  bool show = mPropShowLabel->getBool();

  for (auto const & obj : mObjects) {
    obj.second->updateShowLabel(show);
  }
}

void ZedOdDisplay::updateShowBBox()
{
  bool show = mPropShowBBox->getBool();

  for (auto const & obj : mObjects) {
    obj.second->updateShowBBox(show);
  }
}

void ZedOdDisplay::updateLinkSize()
{
  float val = mPropLinkSize->getFloat();

  for (auto const & obj : mObjects) {
    obj.second->updateLinkSize(val);
  }
}

void ZedOdDisplay::updateJointRadius()
{
  float val = mPropJointRadius->getFloat();

  for (auto const & obj : mObjects) {
    obj.second->updateJointRadius(val);
  }
}

void ZedOdDisplay::updateLabelScale()
{
  float val = mPropLabelScale->getFloat();

  for (auto const & obj : mObjects) {
    obj.second->updateLabelScale(val);
  }
}

}  // namespace displays
}  // namespace rviz_plugin_zed_od

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rviz_plugin_zed_od::displays::ZedOdDisplay,
  rviz_common::Display)
