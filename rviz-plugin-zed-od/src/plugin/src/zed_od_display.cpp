#include "zed_od_display.hpp"

#include <iostream>

namespace rviz_plugin_zed_od
{
namespace displays
{
ZedOdDisplay::ZedOdDisplay() {
    mPropAlpha = new rviz_common::properties::FloatProperty( "Transparency", 0.25f,
                                                             "Structures transparency level [0 -> NONE, 1 -> Invisible]",
                                                             this, SLOT(updateAlpha()));
    mPropAlpha->setMin(0);
    mPropAlpha->setMax(1);

    mPropShowSkeleton = new rviz_common::properties::BoolProperty( "Show Skeletons", true,
                                                                   "Enable/Disable skeletons visualization (if available)",
                                                                   this, SLOT(updateShowSkeleton()));

    mPropShowLabel = new rviz_common::properties::BoolProperty( "Show Labels", true,
                                                                "Enable/Disable ID + Label visualization",
                                                                this, SLOT(updateShowLabel()));

    mPropShowBBox = new rviz_common::properties::BoolProperty( "Show Bounding Boxes", true,
                                                               "Enable/Disable Bounding boxes visualization",
                                                               this, SLOT(updateShowBBox()));
}

ZedOdDisplay::~ZedOdDisplay() {

}

void ZedOdDisplay::onInitialize() {
    MFDClass::onInitialize();
}

void ZedOdDisplay::reset() {
    MFDClass::reset();
}

void ZedOdDisplay::onEnable() {
    MFDClass::onEnable();
}

void ZedOdDisplay::onDisable() {
    MFDClass::onDisable();
}

void ZedOdDisplay::invalidateObjs() {
    for(auto& it : mObjUpdated)
    {
        it.second = false;
    }
}

void ZedOdDisplay::removeNotValidObjs() {
    std::map<int16_t,bool>::iterator it;
    for(it=mObjUpdated.begin(); it!=mObjUpdated.end(); it++)
    {
        int16_t id = it->first;
        if( it->second == false ) {
            auto obj_it = mObjects.find(id);
            if( obj_it != mObjects.end() ) {
                mObjects.erase(obj_it);
            }
        }
    }
}

void ZedOdDisplay::processMessage(zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
    }
    setTransformOk();

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    invalidateObjs();

    for( auto object: msg->objects ) {
        createOrUpdateObject(object);
    }

    removeNotValidObjs();

    mPropShowSkeleton->setHidden(!msg->objects[0].skeleton_available);
}

void ZedOdDisplay::createOrUpdateObject(zed_interfaces::msg::Object& obj) {
    int16_t id = obj.label_id;
    if(id==-1) {
        return;
    }

    mObjUpdated[id] = true;

    auto it = mObjects.find(id);
    if (it == mObjects.end()) {
        objectPtr newObj = std::make_shared<ZedOdInfo>(obj,scene_manager_,scene_node_);
        mObjects[id]=newObj;
        updateAlpha();
        updateShowLabel();
        updateShowBBox();
        updateShowSkeleton();
    } else {
        if(it->second) {
            it->second->updateInfo(obj);
        }
    }
}

void ZedOdDisplay::updateAlpha() {
    float alpha = 1.0-mPropAlpha->getFloat();

    for(auto const& obj : mObjects) {
        obj.second->updateAlpha(alpha);
    }
}

void ZedOdDisplay::updateShowSkeleton() {
    bool show = mPropShowSkeleton->getBool();

    for(auto const& obj : mObjects) {
        obj.second->updateShowSkeleton(show);
    }
}

void ZedOdDisplay::updateShowLabel() {
    bool show = mPropShowLabel->getBool();

    for(auto const& obj : mObjects) {
        obj.second->updateShowLabel(show);
    }
}

void ZedOdDisplay::updateShowBBox() {
    bool show = mPropShowBBox->getBool();

    for(auto const& obj : mObjects) {
        obj.second->updateShowBBox(show);
    }
}

} // namespace displays
} // namespace rviz_plugin_zed_od

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugin_zed_od::displays::ZedOdDisplay, rviz_common::Display)
