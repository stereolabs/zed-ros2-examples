#ifndef ZED_OD_PLUGIN_HPP
#define ZED_OD_PLUGIN_HPP

#include <zed_interfaces/msg/objects_stamped.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include "visibility_control.hpp"

#include <map>

#include "zed_od_info.hpp"

namespace rviz_plugin_zed_od
{
namespace displays
{

typedef std::shared_ptr<ZedOdInfo> objectPtr;

class ZED_OD_PLUGIN_PUBLIC ZedOdDisplay : public
        rviz_common::MessageFilterDisplay<zed_interfaces::msg::ObjectsStamped> {
    Q_OBJECT

public:
    ZedOdDisplay();
    ~ZedOdDisplay();

    void onInitialize() override;
    void reset() override;

private:
    void processMessage(zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg) override;
    void createOrUpdateObject(zed_interfaces::msg::Object& obj);
    void invalidateObjs();
    void removeNotValidObjs();

private slots:
    void updateShowSkeleton();
    void updateShowLabel();
    void updateAlpha();
    void updateShowBBox();
    void updateLinkSize();
    void updateJointRadius();
    void updateLabelScale();

protected:
    /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
    void onEnable() override;
    void onDisable() override;

private:
    rviz_common::properties::FloatProperty* mPropAlpha;
    rviz_common::properties::BoolProperty* mPropShowSkeleton;
    rviz_common::properties::BoolProperty* mPropShowLabel;
    rviz_common::properties::BoolProperty* mPropShowBBox;
    rviz_common::properties::FloatProperty* mPropLinkSize;
    rviz_common::properties::FloatProperty* mPropJointRadius;
    rviz_common::properties::FloatProperty* mPropLabelScale;

    std::map<int16_t,objectPtr> mObjects;
    std::map<int16_t,bool> mObjUpdated;
};

} // namespace displays
} // namespace rviz_plugin_zed_od

#endif // #define ZED_OD_PLUGIN_HPP
