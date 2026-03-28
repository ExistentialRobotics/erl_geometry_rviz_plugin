#pragma once
#ifndef Q_MOC_RUN
    #include <OgreManualObject.h>
    #include <OgreSceneManager.h>
    #include <OgreSceneNode.h>
    #include <erl_geometry_msgs/msg/frontier_array.hpp>
    #include <rclcpp/rclcpp.hpp>
    #include <rviz_common/ros_topic_display.hpp>
    #include <rviz_common/properties/bool_property.hpp>
    #include <rviz_common/properties/color_property.hpp>
    #include <rviz_common/properties/enum_property.hpp>
    #include <rviz_common/properties/float_property.hpp>
    #include <rviz_default_plugins/visibility_control.hpp>
#endif

#include <mutex>
#include <vector>

namespace erl::geometry::rviz_plugin {

    class RVIZ_DEFAULT_PLUGINS_PUBLIC FrontierDisplay
        : public rviz_common::RosTopicDisplay<erl_geometry_msgs::msg::FrontierArray> {
        Q_OBJECT

    private:
        using Super = rviz_common::RosTopicDisplay<erl_geometry_msgs::msg::FrontierArray>;

        std::mutex m_mutex_;

        // cached message data
        struct FrontierData {
            uint32_t id;
            double score;
            std::vector<Ogre::Vector3> vertices;
            std::vector<uint32_t> indices;
        };

        std::vector<FrontierData> m_frontiers_;
        uint8_t m_dim_ = 0;
        bool m_dirty_ = false;
        std_msgs::msg::Header m_header_;

        // Ogre objects
        Ogre::ManualObject *m_manual_object_ = nullptr;
        Ogre::MaterialPtr m_material_;

        // color mode
        enum ColorMode { UNIFORM = 0, BY_ID = 1, BY_SCORE = 2 };

        // properties
        rviz_common::properties::FloatProperty *m_alpha_property_ = nullptr;
        rviz_common::properties::ColorProperty *m_default_color_property_ = nullptr;
        rviz_common::properties::EnumProperty *m_color_mode_property_ = nullptr;
        rviz_common::properties::FloatProperty *m_line_width_property_ = nullptr;
        rviz_common::properties::BoolProperty *m_backface_culling_property_ = nullptr;

    public:
        FrontierDisplay();
        ~FrontierDisplay() override;

        void
        onInitialize() override;
        void
        update(float wall_dt, float ros_dt) override;
        void
        reset() override;

    private Q_SLOTS:
        void
        UpdateAlpha();
        void
        UpdateDefaultColor();
        void
        UpdateColorMode();
        void
        UpdateLineWidth();
        void
        UpdateBackfaceCulling();

    protected:
        void
        onEnable() override;
        void
        onDisable() override;

        void
        processMessage(erl_geometry_msgs::msg::FrontierArray::ConstSharedPtr msg) override;

        void
        BuildVisual();

        Ogre::ColourValue
        ColorForFrontier(size_t index, const FrontierData &frontier) const;

        void
        ComputeNormals(
            const std::vector<Ogre::Vector3> &vertices,
            const std::vector<uint32_t> &indices,
            std::vector<Ogre::Vector3> &normals) const;

        void
        Clear();

        bool
        UpdateFromTf();
    };
}  // namespace erl::geometry::rviz_plugin
