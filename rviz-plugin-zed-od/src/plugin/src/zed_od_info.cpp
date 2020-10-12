
#include "zed_od_info.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include <iostream>

namespace rviz_plugin_zed_od
{
namespace displays
{

// Unique identifier for each object
uint64_t ZedOdInfo::mObjIdx=0;

ZedOdInfo::ZedOdInfo(zed_interfaces::msg::Object& obj,
                     Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node) {

    mSceneManager = scene_manager;
    mParentNode = parent_node;

    mLabelId = obj.label_id;
    calculateColor();

    updateInfo(obj);

    mObjIdx++;
}

ZedOdInfo::~ZedOdInfo() {
    if(mPivotSceneNode) {
        mSceneManager->destroySceneNode(mPivotSceneNode);
    }
    if(mBBoxSceneNode) {
        mSceneManager->destroySceneNode(mBBoxSceneNode);
    }
    if(mSkelSceneNode) {
        mSceneManager->destroySceneNode(mSkelSceneNode);
    }
    if(mSceneNode) {
        mSceneManager->destroySceneNode(mSceneNode);
    }
}

void ZedOdInfo::updateInfo(zed_interfaces::msg::Object &obj) {

    // Check if scene must be created
    bool create = mSceneNode==nullptr;

    // Create node
    if(create) {
        mObjName = std::to_string(obj.label_id);
        mObjName += "-"+obj.label;

        std::string nodeStr = mObjName+std::to_string(mObjIdx);
        mSceneNode = mParentNode->createChildSceneNode(nodeStr.c_str());
    }

    // ----> Pivot and Label
    if(create) {
        std::string nodeStr = std::string("Pivot") + std::to_string(mObjIdx);
        mPivotSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());

        mLabel = std::make_shared<rviz_rendering::MovableText>(mObjName,"Liberation Sans",mTextSize);
        mLabel->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);

        mPivot = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere,mSceneManager,mPivotSceneNode);
        mPivot->setColor(mColor);
        mPivot->setScale(Ogre::Vector3(0.02,0.02,0.02));

        mPivot->getRootNode()->attachObject(mLabel.get());

        mPivotSceneNode->setVisible(mShowLabel);
    }

    // Check if position is valid
    Ogre::Vector3 pos;
    pos[0] = obj.position[0];
    pos[1] = obj.position[1];
    pos[2] = obj.position[2];
    if( pos.isNaN() ) {
        return;
    }

    mPivot->setPosition(pos);
    //std::cout << "pivot: " << pos[0] << "," << pos[1] << ","  << pos[2] << std::endl;
    // <---- Pivot and Label


    // ----> Bounding Box
    if(create) {
        std::string nodeStr = std::string("BBoxes") + std::to_string(mObjIdx);
        mBBoxSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());
    }

    for( int i=0; i<8; i++) {
        Ogre::Vector3 pos;
        zed_interfaces::msg::Keypoint3D corner = obj.bounding_box_3d.corners[i];
        pos[0] = corner.kp[0];
        pos[1] = corner.kp[1];
        pos[2] = corner.kp[2];
        //std::cout << "[" << i << "] corner: " << pos[0] << "," << pos[1] << ","  << pos[2] << std::endl;

        shapePtr sphere;
        if(create) {
            sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere,mSceneManager,mBBoxSceneNode);
            Ogre::Vector3 scale;
            scale[0] = mJointRadius;
            scale[1] = mJointRadius;
            scale[2] = mJointRadius;
            sphere->setScale(scale);
            sphere->setColor(mColor);
            mBBoxCorners.push_back(sphere);
        } else {
            sphere = mBBoxCorners[i];
        }
        if( !pos.isNaN() ) {
            sphere->setPosition(pos);
        }
    }

    for( int i=0; i<4; i++) {
        linePtr line;
        size_t idx=0;

        if(create) {
            line = std::make_shared<rviz_rendering::BillboardLine>(mSceneManager,mBBoxSceneNode);
            line->setColor(mColor.r,mColor.g,mColor.b,mColor.a);
            if(mShowBBox) {
                line->setLineWidth(mLineSize);
            } else {
                line->setLineWidth(0.f);
            }
            mBBoxLines.push_back(line);
        } else {
            line = mBBoxLines[i];
        }

        Ogre::Vector3 start,end;
        idx = i%4;
        zed_interfaces::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
        start[0] = cornerStart.kp[0];
        start[1] = cornerStart.kp[1];
        start[2] = cornerStart.kp[2];
        idx = (i+1)%4;
        zed_interfaces::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
        end[0] = cornerEnd.kp[0];
        end[1] = cornerEnd.kp[1];
        end[2] = cornerEnd.kp[2];
        line->clear();
        line->addPoint(start);
        line->addPoint(end);
    }

    for( int i=0; i<4; i++) {
        linePtr line;
        size_t idx=0;

        if(create) {
            line = std::make_shared<rviz_rendering::BillboardLine>(mSceneManager,mBBoxSceneNode);
            line->setColor(mColor.r,mColor.g,mColor.b,mColor.a);
            if(mShowBBox) {
                line->setLineWidth(mLineSize);
            } else {
                line->setLineWidth(0.f);
            }
            mBBoxLines.push_back(line);
        } else {
            line = mBBoxLines[i+4];
        }

        Ogre::Vector3 start,end;
        idx = i%4+4;
        zed_interfaces::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
        start[0] = cornerStart.kp[0];
        start[1] = cornerStart.kp[1];
        start[2] = cornerStart.kp[2];
        idx = (i+1)%4+4;
        zed_interfaces::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
        end[0] = cornerEnd.kp[0];
        end[1] = cornerEnd.kp[1];
        end[2] = cornerEnd.kp[2];
        line->clear();
        line->addPoint(start);
        line->addPoint(end);
    }

    for( int i=0; i<4; i++) {
        linePtr line;
        size_t idx=0;

        if(create) {
            line = std::make_shared<rviz_rendering::BillboardLine>(mSceneManager,mBBoxSceneNode);
            line->setColor(mColor.r,mColor.g,mColor.b,mColor.a);
            if(mShowBBox) {
                line->setLineWidth(mLineSize);
            } else {
                line->setLineWidth(0.f);
            }
            mBBoxLines.push_back(line);
        } else {
            line = mBBoxLines[i+8];
        }

        Ogre::Vector3 start,end;
        idx = i;
        zed_interfaces::msg::Keypoint3D cornerStart = obj.bounding_box_3d.corners[idx];
        start[0] = cornerStart.kp[0];
        start[1] = cornerStart.kp[1];
        start[2] = cornerStart.kp[2];
        idx = i+4;
        zed_interfaces::msg::Keypoint3D cornerEnd = obj.bounding_box_3d.corners[idx];
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
    if(obj.skeleton_available) {
        if(create) {
            std::string nodeStr = std::string("Skeleton") + std::to_string(mObjIdx);
            mSkelSceneNode = mSceneNode->createChildSceneNode(nodeStr.c_str());
        }

        size_t size = obj.skeleton_3d.keypoints.size();

        for( int i=0; i<size; i++ ) {
            Ogre::Vector3 pos;
            zed_interfaces::msg::Keypoint3D joint = obj.skeleton_3d.keypoints[i];
            pos[0] = joint.kp[0];
            pos[1] = joint.kp[1];
            pos[2] = joint.kp[2];

            shapePtr sphere;
            if(create) {
                sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere,mSceneManager,mSkelSceneNode);
                Ogre::Vector3 scale;
                scale[0] = mJointRadius/2.f;
                scale[1] = mJointRadius/2.f;
                scale[2] = mJointRadius/2.f;
                sphere->setScale(scale);
                sphere->setColor(mColor);
                mSkelJoints.push_back(sphere);
            } else {
                sphere = mSkelJoints[i];
            }

            if( qIsNaN(pos[0]) ) {
                sphere->setScale(Ogre::Vector3(0,0,0));
                sphere->setPosition(Ogre::Vector3(0,0,0));
            } else {
                sphere->setScale(Ogre::Vector3(mJointRadius/2.f,mJointRadius/2.f,mJointRadius/2.f));
                if( !pos.isNaN() ) {
                    sphere->setPosition(pos);
                }
            }
        }

        size_t idx=0;
        for (auto& limb : BODY_BONES) {
            linePtr link;

            if(create) {
                link = std::make_shared<rviz_rendering::BillboardLine>(mSceneManager,mSkelSceneNode);
                link->setColor(mColor.r,mColor.g,mColor.b,mColor.a);
                link->setLineWidth(mLineSize/2.f);
                mSkelLinks.push_back(link);
            } else {
                link = mSkelLinks[idx];
            }
            idx++;

            Ogre::Vector3 start,end;
            start[0] = obj.skeleton_3d.keypoints[(int)limb.first].kp[0];
            start[1] = obj.skeleton_3d.keypoints[(int)limb.first].kp[1];
            start[2] = obj.skeleton_3d.keypoints[(int)limb.first].kp[2];

            end[0] = obj.skeleton_3d.keypoints[(int)limb.second].kp[0];
            end[1] = obj.skeleton_3d.keypoints[(int)limb.second].kp[1];
            end[2] = obj.skeleton_3d.keypoints[(int)limb.second].kp[2];
            if (qIsNaN(start[0]) || qIsNaN(end[0]) || !mShowSkel) {
                link->setLineWidth(0.f);
            } else {
                link->setLineWidth(mLineSize/2.f);
                link->clear();
                link->addPoint(start);
                link->addPoint(end);
            }
        }
    }
    // <---- Skeleton
}

void ZedOdInfo::calculateColor() {
    quint8 r = (mLabelId+1)*30;
    quint8 g = (mLabelId+1)*60;
    quint8 b = (mLabelId+1)*120;

    mColor.r = static_cast<float>(r)/255.f;
    mColor.g = static_cast<float>(g)/255.f;;
    mColor.b = static_cast<float>(b)/255.f;;
    mColor.a = mAlpha;

    //std::cout << "RGB: " << (int)r << "," << (int)g << "," << (int)b << " (" << mColor.r << "," << mColor.g << "," << mColor.b << ")" << std::endl;
}

void ZedOdInfo::updateAlpha(float alpha) {
    mAlpha = alpha;

    calculateColor();

    for( auto sphere : mBBoxCorners ) {
        sphere->setColor(mColor);
    }

    for( auto line : mBBoxLines ) {
        line->setColor(mColor.r,mColor.g,mColor.b,mColor.a);
    }
}

void ZedOdInfo::updateShowLabel(bool show) {
    mShowLabel = show;

    if(mPivotSceneNode) {
        mPivotSceneNode->setVisible(mShowLabel);
    }
}

void ZedOdInfo::updateShowBBox(bool show) {
    mShowBBox = show;

    if(mBBoxSceneNode) {
        mBBoxSceneNode->setVisible(mShowBBox);
    }
}

void ZedOdInfo::updateShowSkeleton(bool show) {
    mShowSkel = show;

    if(mSkelSceneNode) {
        mSkelSceneNode->setVisible(mShowSkel);
    }
}

} // namespace displays
} // namespace rviz_plugin_zed_od
