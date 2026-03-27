#pragma once
#ifndef Q_MOC_RUN
    #include <message_filters/subscriber.h>
    #include <ros/ros.h>
    #include <rviz/display.h>
    #include <rviz/properties/bool_property.h>
    #include <rviz/properties/color_property.h>
    #include <rviz/properties/enum_property.h>
    #include <rviz/properties/float_property.h>
    #include <rviz/properties/int_property.h>
    #include <rviz/properties/ros_topic_property.h>
    #include <rviz/properties/string_property.h>
    #include <erl_geometry_msgs/MeshMsg.h>
    #include <std_msgs/Header.h>

    #include <OGRE/OgreManualObject.h>
    #include <OGRE/OgreSceneManager.h>
    #include <OGRE/OgreSceneNode.h>
#endif

#include <mutex>
#include <string>
#include <vector>

namespace erl::geometry::rviz_plugin {

    class MeshDisplay : public rviz::Display {
        Q_OBJECT

    private:
        std::shared_ptr<message_filters::Subscriber<erl_geometry_msgs::MeshMsg>> m_sub_;
        std::mutex m_mutex_;

        // mesh data buffers
        std::vector<Ogre::Vector3> m_vertices_;
        std::vector<uint32_t> m_indices_;
        std::vector<Ogre::Vector3> m_normals_;
        std::vector<Ogre::ColourValue> m_colors_;
        bool m_mesh_dirty_ = false;

        // Ogre objects
        Ogre::ManualObject* m_manual_object_ = nullptr;
        Ogre::MaterialPtr m_material_;

        // properties
        rviz::IntProperty* m_queue_size_property_ = nullptr;
        rviz::RosTopicProperty* m_topic_property_ = nullptr;
        rviz::StringProperty* m_ply_file_property_ = nullptr;
        rviz::FloatProperty* m_alpha_property_ = nullptr;
        rviz::ColorProperty* m_default_color_property_ = nullptr;
        rviz::BoolProperty* m_use_normals_property_ = nullptr;
        rviz::BoolProperty* m_backface_culling_property_ = nullptr;

        uint32_t m_queue_size_ = 5;
        uint32_t m_messages_received_ = 0;
        std_msgs::Header m_header_;

    public:
        MeshDisplay();
        ~MeshDisplay() override;

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
        UpdatePlyFile();
        void
        UpdateAlpha();
        void
        UpdateDefaultColor();
        void
        UpdateUseNormals();
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
        IncomingMessageCallback(const erl_geometry_msgs::MeshMsg::ConstPtr& msg);

        void
        LoadPlyFile(const std::string& file_path);

        void
        BuildMesh();

        void
        ComputeNormals();

        void
        Clear();

        bool
        UpdateFromTf();
    };
}  // namespace erl::geometry::rviz_plugin
