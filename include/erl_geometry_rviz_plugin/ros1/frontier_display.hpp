#pragma once
#ifndef Q_MOC_RUN
    #include <erl_geometry_msgs/FrontierArray.h>
    #include <message_filters/subscriber.h>
    #include <OGRE/OgreManualObject.h>
    #include <OGRE/OgreSceneManager.h>
    #include <OGRE/OgreSceneNode.h>
    #include <ros/ros.h>
    #include <rviz/display.h>
    #include <rviz/properties/bool_property.h>
    #include <rviz/properties/color_property.h>
    #include <rviz/properties/enum_property.h>
    #include <rviz/properties/float_property.h>
    #include <rviz/properties/int_property.h>
    #include <rviz/properties/ros_topic_property.h>
    #include <std_msgs/Header.h>
#endif

#include <mutex>
#include <vector>

namespace erl::geometry::rviz_plugin {

    class FrontierDisplay : public rviz::Display {
        Q_OBJECT

    private:
        std::shared_ptr<message_filters::Subscriber<erl_geometry_msgs::FrontierArray>> m_sub_;
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
        std_msgs::Header m_header_;

        // Ogre objects
        Ogre::ManualObject *m_manual_object_ = nullptr;
        Ogre::MaterialPtr m_material_;

        // color mode
        enum ColorMode { UNIFORM = 0, BY_ID = 1, BY_SCORE = 2 };

        // properties
        rviz::IntProperty *m_queue_size_property_ = nullptr;
        rviz::RosTopicProperty *m_topic_property_ = nullptr;
        rviz::FloatProperty *m_alpha_property_ = nullptr;
        rviz::ColorProperty *m_default_color_property_ = nullptr;
        rviz::EnumProperty *m_color_mode_property_ = nullptr;
        rviz::FloatProperty *m_line_width_property_ = nullptr;
        rviz::BoolProperty *m_backface_culling_property_ = nullptr;

        uint32_t m_queue_size_ = 5;

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
        UpdateQueueSize();
        void
        UpdateTopic();
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
        Subscribe();
        void
        Unsubscribe();

        void
        IncomingMessageCallback(const erl_geometry_msgs::FrontierArray::ConstPtr &msg);

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
