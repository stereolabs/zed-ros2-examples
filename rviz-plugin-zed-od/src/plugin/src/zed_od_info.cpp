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

#include "zed_od_info.hpp"

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <iostream>

namespace rviz_plugin_zed_od
{
namespace displays
{
const Ogre::ColourValue COLOR_TRACK_OFF(0.0f, 0.0f, 0.0f);
const Ogre::ColourValue COLOR_TRACK_OK(0.0f, 1.0f, 0.0f);
const Ogre::ColourValue COLOR_TRACK_SEARCH(0.5f, 0.35f, 0.0f);
const Ogre::ColourValue COLOR_TRACK_TERMINATED(1.0f, 1.0f, 1.0f);

// Unique identifier for each object
uint64_t ZedOdInfo::mObjIdx = 0;

ZedOdInfo::ZedOdInfo(
  zed_msgs::msg::Object & obj,
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_node)
{
  mSceneManager = scene_manager;
  mParentNode = parent_node;

  mLabelId = obj.label_id;

  mObjIdx++;

  // std::cout << "New object: " << mObjIdx << std::endl;

  calculateColor();

  updateInfo(obj);
}

ZedOdInfo::~ZedOdInfo()
{
  if (mPivotSceneNode) {
    mSceneManager->destroySceneNode(mPivotSceneNode);
  }
  if (mBBoxSceneNode) {
    mSceneManager->destroySceneNode(mBBoxSceneNode);
  }
  if (mSkelSceneNode) {
    mSceneManager->destroySceneNode(mSkelSceneNode);
  }
  if (mSceneNode) {
    mSceneManager->destroySceneNode(mSceneNode);
  }
}

void ZedOdInfo::updateInfo(zed_msgs::msg::Object & obj)
{
  // Check if scene must be created
  bool create = (mSceneNode == nullptr);

  // Create node
  if (create) {
    if (obj.skeleton_available) {
      mObjName = "Person";
      if (obj.label_id != -1) {
        mObjName += "-";
        mObjName += std::to_string(obj.label_id);
      }
    } else {
      if (obj.label_id != -1) {
        mObjName = std::to_string(obj.label_id);
        mObjName += "-";
      }
      mObjName += obj.label;
      if (obj.sublabel != obj.label) {
        mObjName += " [";
        mObjName += obj.sublabel;
        mObjName += "]";
      }
    }

    std::string nodeStr = std::to_string(mObjIdx) + "-" + mObjName;
    // std::cout << "New node:" << nodeStr << std::endl;
    mSceneNode = mParentNode->createChildSceneNode(nodeStr.c_str());
  }

  // ----> Pivot and Label
  if (create) {
    std::string nodeStr = std::string("Pivot") + std::to_string(mObjIdx);
    mPivotSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());

    mLabel = std::make_shared<rviz_rendering::MovableText>(
      mObjName, "Liberation Sans", mLabelScale);
    mLabel->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER,
      rviz_rendering::MovableText::V_CENTER);

    mPivot = std::make_shared<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, mSceneManager, mPivotSceneNode);
    mPivot->setColor(mColorBBox);
    mPivot->setScale(Ogre::Vector3(0.02, 0.02, 0.02));

    mPivot->getRootNode()->attachObject(mLabel.get());

    mPivotSceneNode->setVisible(mShowLabel);
  }

  // Check if position is valid
  Ogre::Vector3 pos;
  pos[0] = obj.position[0];
  pos[1] = obj.position[1];
  pos[2] = obj.position[2];
  if (pos.isNaN()) {
    return;
  }

  mPivot->setPosition(pos);
  // std::cout << "pivot: " << pos[0] << "," << pos[1] << ","  << pos[2] <<
  // std::endl;
  // <---- Pivot and Label

  // ----> Bounding Box
  if (create) {
    std::string nodeStr = std::string("BBoxes") + std::to_string(mObjIdx);
    mBBoxSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());
  }

  for (int i = 0; i < 8; i++) {
    Ogre::Vector3 pos;
    zed_msgs::msg::Keypoint3D corner = obj.bounding_box_3d.corners[i];
    pos[0] = corner.kp[0];
    pos[1] = corner.kp[1];
    pos[2] = corner.kp[2];
    // std::cout << "[" << i << "] corner: " <<
    // pos[0] << "," << pos[1] << ","  << pos[2] << std::endl;

    shapePtr sphere;
    if (create) {
      sphere = std::make_shared<rviz_rendering::Shape>(
        rviz_rendering::Shape::Sphere, mSceneManager, mBBoxSceneNode);
      Ogre::Vector3 scale;
      scale[0] = mJointRadius;
      scale[1] = mJointRadius;
      scale[2] = mJointRadius;
      sphere->setScale(scale);
      // sphere->setColor(mColorBBox);
      mBBoxCorners.push_back(sphere);
    } else {
      sphere = mBBoxCorners[i];
    }
    if (!pos.isNaN()) {
      sphere->setPosition(pos);
    }
    switch (obj.tracking_state) {
      case 0:
        sphere->setColor(COLOR_TRACK_OFF);
        // std::cout << "[" << i << "] Tracking OFF" << std::endl;
        break;
      case 1:
        sphere->setColor(COLOR_TRACK_OK);
        // std::cout << "[" << i << "] Tracking OK" << std::endl;
        break;
      case 2:
        sphere->setColor(COLOR_TRACK_SEARCH);
        // std::cout << "[" << i << "] Tracking SEARCHING" << std::endl;
        break;
      case 3:
        sphere->setColor(COLOR_TRACK_TERMINATED);
        // std::cout << "[" << i << "] Tracking TERMINATED" << std::endl;
        break;
    }
  }

  for (int i = 0; i < 4; i++) {
    linePtr line;
    size_t idx = 0;

    if (create) {
      line = std::make_shared<rviz_rendering::BillboardLine>(
        mSceneManager,
        mBBoxSceneNode);
      line->setColor(mColorBBox.r, mColorBBox.g, mColorBBox.b, mColorBBox.a);
      if (mShowBBox) {
        line->setLineWidth(mLinkSize);
      } else {
        line->setLineWidth(0.f);
      }
      mBBoxLines.push_back(line);
    } else {
      line = mBBoxLines[i];
    }

    Ogre::Vector3 start, end;
    idx = i % 4;
    zed_msgs::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
    start[0] = cornerStart.kp[0];
    start[1] = cornerStart.kp[1];
    start[2] = cornerStart.kp[2];
    idx = (i + 1) % 4;
    zed_msgs::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
    end[0] = cornerEnd.kp[0];
    end[1] = cornerEnd.kp[1];
    end[2] = cornerEnd.kp[2];
    line->clear();
    line->addPoint(start);
    line->addPoint(end);
  }

  for (int i = 0; i < 4; i++) {
    linePtr line;
    size_t idx = 0;

    if (create) {
      line = std::make_shared<rviz_rendering::BillboardLine>(
        mSceneManager,
        mBBoxSceneNode);
      line->setColor(mColorBBox.r, mColorBBox.g, mColorBBox.b, mColorBBox.a);
      if (mShowBBox) {
        line->setLineWidth(mLinkSize);
      } else {
        line->setLineWidth(0.f);
      }
      mBBoxLines.push_back(line);
    } else {
      line = mBBoxLines[i + 4];
    }

    Ogre::Vector3 start, end;
    idx = i % 4 + 4;
    zed_msgs::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
    start[0] = cornerStart.kp[0];
    start[1] = cornerStart.kp[1];
    start[2] = cornerStart.kp[2];
    idx = (i + 1) % 4 + 4;
    zed_msgs::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
    end[0] = cornerEnd.kp[0];
    end[1] = cornerEnd.kp[1];
    end[2] = cornerEnd.kp[2];
    line->clear();
    line->addPoint(start);
    line->addPoint(end);
  }

  for (int i = 0; i < 4; i++) {
    linePtr line;
    size_t idx = 0;

    if (create) {
      line = std::make_shared<rviz_rendering::BillboardLine>(
        mSceneManager,
        mBBoxSceneNode);
      line->setColor(mColorBBox.r, mColorBBox.g, mColorBBox.b, mColorBBox.a);
      if (mShowBBox) {
        line->setLineWidth(mLinkSize);
      } else {
        line->setLineWidth(0.f);
      }
      mBBoxLines.push_back(line);
    } else {
      line = mBBoxLines[i + 8];
    }

    Ogre::Vector3 start, end;
    idx = i;
    zed_msgs::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
    start[0] = cornerStart.kp[0];
    start[1] = cornerStart.kp[1];
    start[2] = cornerStart.kp[2];
    idx = i + 4;
    zed_msgs::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
    end[0] = cornerEnd.kp[0];
    end[1] = cornerEnd.kp[1];
    end[2] = cornerEnd.kp[2];
    line->clear();
    line->addPoint(start);
    line->addPoint(end);
  }

  mBBoxSceneNode->setVisible(mShowBBox);
  // <---- Bounding Box

  // ----> Skeleton
  if (obj.skeleton_available) {
    if (create) {
      std::string nodeStr = std::string("Skeleton") + std::to_string(mObjIdx);
      mSkelSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());
    }

    size_t size = obj.skeleton_3d.keypoints.size();

    for (int i = 0; i < size; i++) {
      Ogre::Vector3 pos;
      zed_msgs::msg::Keypoint3D joint = obj.skeleton_3d.keypoints[i];
      pos[0] = joint.kp[0];
      pos[1] = joint.kp[1];
      pos[2] = joint.kp[2];

      shapePtr sphere;
      if (create) {
        sphere = std::make_shared<rviz_rendering::Shape>(
          rviz_rendering::Shape::Sphere, mSceneManager, mSkelSceneNode);
        Ogre::Vector3 scale;
        scale[0] = mJointRadius * mSkelScale;
        scale[1] = mJointRadius * mSkelScale;
        scale[2] = mJointRadius * mSkelScale;
        sphere->setScale(scale);
        sphere->setColor(mColorSkel);
        mSkelJoints.push_back(sphere);
      } else {
        sphere = mSkelJoints[i];
      }

      if (qIsNaN(pos[0])) {
        sphere->setScale(Ogre::Vector3(0, 0, 0));
        sphere->setPosition(Ogre::Vector3(0, 0, 0));
      } else {
        sphere->setScale(
          Ogre::Vector3(
            mJointRadius * mSkelScale,
            mJointRadius * mSkelScale,
            mJointRadius * mSkelScale));
        if (!pos.isNaN()) {
          sphere->setPosition(pos);
        }
      }
    }

    if (obj.body_format == 0) {
      size_t idx = 0;
      for (auto & limb : BODY_18_BONES) {
        linePtr link;

        if (create) {
          link = std::make_shared<rviz_rendering::BillboardLine>(
            mSceneManager, mSkelSceneNode);
          link->setColor(
            mColorSkel.r, mColorSkel.g, mColorSkel.b,
            mColorSkel.a);
          link->setLineWidth(mLinkSize * mSkelScale);
          mSkelLinks.push_back(link);
        } else {
          link = mSkelLinks[idx];
        }
        idx++;

        Ogre::Vector3 start, end;
        start[0] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[0];
        start[1] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[1];
        start[2] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[2];

        end[0] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[0];
        end[1] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[1];
        end[2] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[2];
        if (qIsNaN(start[0]) || qIsNaN(end[0]) || !mShowSkel) {
          link->setLineWidth(0.f);
        } else {
          link->setLineWidth(mLinkSize * mSkelScale);
          link->clear();
          link->addPoint(start);
          link->addPoint(end);
        }
      }
    } else if (obj.body_format == 1) {
      size_t idx = 0;
      for (auto & limb : BODY_34_BONES) {
        linePtr link;

        if (create) {
          link = std::make_shared<rviz_rendering::BillboardLine>(
            mSceneManager, mSkelSceneNode);
          link->setColor(
            mColorSkel.r, mColorSkel.g, mColorSkel.b,
            mColorSkel.a);
          link->setLineWidth(mLinkSize * mSkelScale);
          mSkelLinks.push_back(link);
        } else {
          link = mSkelLinks[idx];
        }
        idx++;

        Ogre::Vector3 start, end;
        start[0] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[0];
        start[1] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[1];
        start[2] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[2];

        end[0] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[0];
        end[1] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[1];
        end[2] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[2];
        if (qIsNaN(start[0]) || qIsNaN(end[0]) || !mShowSkel) {
          link->setLineWidth(0.f);
        } else {
          link->setLineWidth(mLinkSize * mSkelScale);
          link->clear();
          link->addPoint(start);
          link->addPoint(end);
        }
      }
    } else if (obj.body_format == 2) {
      size_t idx = 0;
      for (auto & limb : BODY_38_BONES) {
        linePtr link;

        if (create) {
          link = std::make_shared<rviz_rendering::BillboardLine>(
            mSceneManager, mSkelSceneNode);
          link->setColor(
            mColorSkel.r, mColorSkel.g, mColorSkel.b,
            mColorSkel.a);
          link->setLineWidth(mLinkSize * mSkelScale);
          mSkelLinks.push_back(link);
        } else {
          link = mSkelLinks[idx];
        }
        idx++;

        Ogre::Vector3 start, end;
        start[0] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[0];
        start[1] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[1];
        start[2] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[2];

        end[0] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[0];
        end[1] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[1];
        end[2] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[2];
        if (qIsNaN(start[0]) || qIsNaN(end[0]) || !mShowSkel) {
          link->setLineWidth(0.f);
        } else {
          link->setLineWidth(mLinkSize * mSkelScale);
          link->clear();
          link->addPoint(start);
          link->addPoint(end);
        }
      }
    } else if (obj.body_format == 3) {
      size_t idx = 0;
      for (auto & limb : BODY_70_BONES) {
        linePtr link;

        if (create) {
          link = std::make_shared<rviz_rendering::BillboardLine>(
            mSceneManager, mSkelSceneNode);
          link->setColor(
            mColorSkel.r, mColorSkel.g, mColorSkel.b,
            mColorSkel.a);
          link->setLineWidth(mLinkSize * mSkelScale);
          mSkelLinks.push_back(link);
        } else {
          link = mSkelLinks[idx];
        }
        idx++;

        Ogre::Vector3 start, end;
        start[0] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[0];
        start[1] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[1];
        start[2] =
          obj.skeleton_3d.keypoints[static_cast<int>(limb.first)].kp[2];

        end[0] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[0];
        end[1] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[1];
        end[2] = obj.skeleton_3d.keypoints[static_cast<int>(limb.second)].kp[2];
        if (qIsNaN(start[0]) || qIsNaN(end[0]) || !mShowSkel) {
          link->setLineWidth(0.f);
        } else {
          link->setLineWidth(mLinkSize * mSkelScale);
          link->clear();
          link->addPoint(start);
          link->addPoint(end);
        }
      }
    }
  }
  // <---- Skeleton
}

void ZedOdInfo::calculateColor()
{
  int idx = (mLabelId != -1) ? mLabelId : 0;

  quint8 r = (idx + 2) * 30;
  quint8 g = (idx + 2) * 60;
  quint8 b = (idx + 2) * 120;

  mColorBBox.r = static_cast<float>(r) / 255.f;
  mColorBBox.g = static_cast<float>(g) / 255.f;
  mColorBBox.b = static_cast<float>(b) / 255.f;
  mColorBBox.a = mAlpha;

  mColorSkel.r = static_cast<float>(r + 2 * mSkelColOffset) / 255.f;
  mColorSkel.g = static_cast<float>(g + mSkelColOffset) / 255.f;
  mColorSkel.b = static_cast<float>(b - mSkelColOffset) / 255.f;
  mColorSkel.a = mAlpha;
}

void ZedOdInfo::updateAlpha(float alpha)
{
  mAlpha = alpha;

  calculateColor();

  for (auto line : mBBoxLines) {
    line->setColor(mColorBBox.r, mColorBBox.g, mColorBBox.b, mColorBBox.a);
  }

  for (auto sphere : mSkelJoints) {
    sphere->setColor(mColorSkel.r, mColorSkel.g, mColorSkel.b, mColorSkel.a);
  }

  for (auto line : mSkelLinks) {
    line->setColor(mColorSkel.r, mColorSkel.g, mColorSkel.b, mColorSkel.a);
  }
}

void ZedOdInfo::updateLinkSize(float newval)
{
  mLinkSize = newval;

  for (auto line : mBBoxLines) {
    line->setLineWidth(mLinkSize);
  }

  for (auto line : mSkelLinks) {
    line->setLineWidth(mLinkSize * mSkelScale);
  }
}

void ZedOdInfo::updateJointRadius(float newval)
{
  mJointRadius = newval;

  Ogre::Vector3 scale_bbox;
  scale_bbox[0] = mJointRadius;
  scale_bbox[1] = mJointRadius;
  scale_bbox[2] = mJointRadius;

  Ogre::Vector3 scale_skel;
  scale_skel[0] = mJointRadius * mSkelScale;
  scale_skel[1] = mJointRadius * mSkelScale;
  scale_skel[2] = mJointRadius * mSkelScale;

  for (auto sphere : mBBoxCorners) {
    sphere->setScale(scale_bbox);
  }

  for (auto sphere : mSkelJoints) {
    sphere->setScale(scale_skel);
  }
}

void ZedOdInfo::updateLabelScale(float newval)
{
  mLabelScale = newval;

  mLabel->setCharacterHeight(mLabelScale);
}

void ZedOdInfo::updateShowLabel(bool show)
{
  mShowLabel = show;

  if (mPivotSceneNode) {
    mPivotSceneNode->setVisible(mShowLabel);
  }
}

void ZedOdInfo::updateShowBBox(bool show)
{
  mShowBBox = show;

  if (mBBoxSceneNode) {
    mBBoxSceneNode->setVisible(mShowBBox);
  }
}

void ZedOdInfo::updateShowSkeleton(bool show)
{
  mShowSkel = show;

  if (mSkelSceneNode) {
    mSkelSceneNode->setVisible(mShowSkel);
  }
}

}  // namespace displays
}  // namespace rviz_plugin_zed_od
