#pragma once
#ifndef Q_MOC_RUN
    #include <erl_geometry_msgs/msg/mesh_msg.hpp>
    #include <OgreManualObject.h>
    #include <OgreSceneManager.h>
    #include <OgreSceneNode.h>
    #include <rclcpp/rclcpp.hpp>
    #include <rviz_common/properties/bool_property.hpp>
    #include <rviz_common/properties/color_property.hpp>
    #include <rviz_common/properties/enum_property.hpp>
    #include <rviz_common/properties/float_property.hpp>
    #include <rviz_common/properties/int_property.hpp>
    #include <rviz_common/properties/string_property.hpp>
    #include <rviz_common/ros_topic_display.hpp>
    #include <rviz_default_plugins/visibility_control.hpp>
#endif

#include <mutex>
#include <string>
#include <vector>

namespace erl::geometry::rviz_plugin {

    class RVIZ_DEFAULT_PLUGINS_PUBLIC MeshDisplay
        : public rviz_common::RosTopicDisplay<erl_geometry_msgs::msg::MeshMsg> {
        Q_OBJECT

    private:
        using Super = rviz_common::RosTopicDisplay<erl_geometry_msgs::msg::MeshMsg>;

        std::mutex m_mutex_;

        // mesh data buffers
        uint8_t m_dim_ = 3;  // 2 = line segments, 3 = triangles
        std::vector<Ogre::Vector3> m_vertices_;
        std::vector<uint32_t> m_indices_;
        std::vector<Ogre::Vector3> m_normals_;
        std::vector<Ogre::ColourValue> m_colors_;
        bool m_mesh_dirty_ = false;

        // Ogre objects
        Ogre::ManualObject *m_manual_object_ = nullptr;
        Ogre::MaterialPtr m_material_;

        // properties
        rviz_common::properties::StringProperty *m_ply_file_property_ = nullptr;
        rviz_common::properties::FloatProperty *m_alpha_property_ = nullptr;
        rviz_common::properties::ColorProperty *m_default_color_property_ = nullptr;
        rviz_common::properties::BoolProperty *m_use_normals_property_ = nullptr;
        rviz_common::properties::BoolProperty *m_backface_culling_property_ = nullptr;

        std_msgs::msg::Header m_header_;

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
        processMessage(erl_geometry_msgs::msg::MeshMsg::ConstSharedPtr msg) override;

        void
        LoadPlyFile(const std::string &file_path);

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
