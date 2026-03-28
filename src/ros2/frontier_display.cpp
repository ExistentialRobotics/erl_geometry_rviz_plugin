#include "erl_geometry_rviz_plugin/ros2/frontier_display.hpp"

#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/visualization_manager.hpp>

#include <cmath>

namespace erl::geometry::rviz_plugin {

    FrontierDisplay::FrontierDisplay()
        : Super() {

        m_color_mode_property_ = new rviz_common::properties::EnumProperty(
            "Color Mode",
            "Uniform",
            "How to color each frontier.",
            this,
            SLOT(UpdateColorMode()));
        m_color_mode_property_->addOption("Uniform", UNIFORM);
        m_color_mode_property_->addOption("By ID", BY_ID);
        m_color_mode_property_->addOption("By Score", BY_SCORE);

        m_default_color_property_ = new rviz_common::properties::ColorProperty(
            "Color",
            QColor(0, 200, 100),
            "Color used in Uniform mode.",
            this,
            SLOT(UpdateDefaultColor()));

        m_alpha_property_ = new rviz_common::properties::FloatProperty(
            "Alpha",
            1.0,
            "Transparency.",
            this,
            SLOT(UpdateAlpha()));
        m_alpha_property_->setMin(0.0);
        m_alpha_property_->setMax(1.0);

        m_line_width_property_ = new rviz_common::properties::FloatProperty(
            "Line Width",
            2.0,
            "Width for 2D frontier line segments (pixels). Ignored for 3D.",
            this,
            SLOT(UpdateLineWidth()));
        m_line_width_property_->setMin(0.5);
        m_line_width_property_->setMax(50.0);

        m_backface_culling_property_ = new rviz_common::properties::BoolProperty(
            "Backface Culling",
            false,
            "Cull back faces for 3D frontiers.",
            this,
            SLOT(UpdateBackfaceCulling()));
    }

    FrontierDisplay::~FrontierDisplay() {
        Clear();
        if (m_manual_object_) {
            scene_manager_->destroyManualObject(m_manual_object_);
            m_manual_object_ = nullptr;
        }
        if (m_material_) {
            Ogre::MaterialManager::getSingleton().remove(m_material_);
            m_material_.reset();
        }
    }

    void
    FrontierDisplay::onInitialize() {
        Super::onInitialize();

        static int material_counter = 0;
        std::string material_name = "FrontierDisplayMaterial_" + std::to_string(material_counter++);
        m_material_ = Ogre::MaterialManager::getSingleton().create(
            material_name,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        m_material_->setReceiveShadows(false);
        m_material_->getTechnique(0)->setLightingEnabled(false);
        m_material_->getTechnique(0)->getPass(0)->setVertexColourTracking(
            Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
        m_material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
        m_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        m_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);

        m_manual_object_ = scene_manager_->createManualObject();
        scene_node_->attachObject(m_manual_object_);
    }

    void
    FrontierDisplay::update(float wall_dt, float ros_dt) {
        if (m_dirty_) {
            std::lock_guard<std::mutex> lock(m_mutex_);
            BuildVisual();
            m_dirty_ = false;
        }
        UpdateFromTf();
        Super::update(wall_dt, ros_dt);
    }

    void
    FrontierDisplay::reset() {
        Clear();
        Super::reset();
    }

    void
    FrontierDisplay::UpdateAlpha() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_dirty_ = true;
    }

    void
    FrontierDisplay::UpdateDefaultColor() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_dirty_ = true;
    }

    void
    FrontierDisplay::UpdateColorMode() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_dirty_ = true;
    }

    void
    FrontierDisplay::UpdateLineWidth() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_dirty_ = true;
    }

    void
    FrontierDisplay::UpdateBackfaceCulling() {
        if (!m_material_) { return; }
        m_material_->getTechnique(0)->getPass(0)->setCullingMode(
            m_backface_culling_property_->getBool() ? Ogre::CULL_CLOCKWISE : Ogre::CULL_NONE);
        context_->queueRender();
    }

    void
    FrontierDisplay::onEnable() {
        scene_node_->setVisible(true);
        Super::onEnable();
    }

    void
    FrontierDisplay::onDisable() {
        scene_node_->setVisible(false);
        Super::onDisable();
    }

    void
    FrontierDisplay::processMessage(erl_geometry_msgs::msg::FrontierArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(m_mutex_);

        m_header_ = msg->header;
        m_dim_ = msg->dim;

        if (m_dim_ != 2 && m_dim_ != 3) {
            setStatusStd(
                rviz_common::properties::StatusProperty::Error,
                "Message",
                "Unsupported dim=" + std::to_string(m_dim_) + ". Expected 2 or 3.");
            return;
        }

        m_frontiers_.resize(msg->frontiers.size());
        for (size_t fi = 0; fi < msg->frontiers.size(); ++fi) {
            const auto &src = msg->frontiers[fi];
            auto &dst = m_frontiers_[fi];

            dst.id = src.id;
            dst.score = src.score;

            dst.vertices.resize(src.vertices.size());
            for (size_t i = 0; i < src.vertices.size(); ++i) {
                dst.vertices[i] = Ogre::Vector3(
                    static_cast<float>(src.vertices[i].x),
                    static_cast<float>(src.vertices[i].y),
                    static_cast<float>(src.vertices[i].z));
            }

            dst.indices.assign(src.indices.begin(), src.indices.end());

            // validate index count
            if (m_dim_ == 2 && dst.indices.size() % 2 != 0) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Warn,
                    "Message",
                    "Frontier " + std::to_string(dst.id) +
                        ": indices count is not a multiple of 2.");
            } else if (m_dim_ == 3 && dst.indices.size() % 3 != 0) {
                setStatusStd(
                    rviz_common::properties::StatusProperty::Warn,
                    "Message",
                    "Frontier " + std::to_string(dst.id) +
                        ": indices count is not a multiple of 3.");
            }
        }

        m_dirty_ = true;
        setStatusStd(
            rviz_common::properties::StatusProperty::Ok,
            "Message",
            std::to_string(m_frontiers_.size()) + " frontier(s), dim=" + std::to_string(m_dim_));
    }

    void
    FrontierDisplay::BuildVisual() {
        if (!m_manual_object_) { return; }
        m_manual_object_->clear();
        if (m_frontiers_.empty() || m_dim_ == 0) { return; }

        float alpha = m_alpha_property_->getFloat();
        m_material_->getTechnique(0)->getPass(0)->setSceneBlending(
            alpha < 1.0f ? Ogre::SBT_TRANSPARENT_ALPHA : Ogre::SBT_REPLACE);
        m_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(alpha >= 1.0f);

        bool is_3d = (m_dim_ == 3);
        auto render_op = is_3d ? Ogre::RenderOperation::OT_TRIANGLE_LIST
                               : Ogre::RenderOperation::OT_LINE_LIST;

        if (!is_3d) {
            float line_width = m_line_width_property_->getFloat();
            m_manual_object_->begin(m_material_->getName(), render_op);
            m_manual_object_->end();
            // ManualObject doesn't support line width directly; we set it on the material
            m_material_->getTechnique(0)->getPass(0)->setPointSize(line_width);
            m_manual_object_->clear();
        }

        m_material_->getTechnique(0)->setLightingEnabled(is_3d);

        for (size_t fi = 0; fi < m_frontiers_.size(); ++fi) {
            const auto &frontier = m_frontiers_[fi];
            if (frontier.vertices.empty() || frontier.indices.empty()) { continue; }

            Ogre::ColourValue color = ColorForFrontier(fi, frontier);
            color.a = alpha;

            m_manual_object_->begin(m_material_->getName(), render_op);

            if (is_3d) {
                // compute per-vertex normals for lighting
                std::vector<Ogre::Vector3> normals;
                ComputeNormals(frontier.vertices, frontier.indices, normals);
                bool has_normals = normals.size() == frontier.vertices.size();

                for (size_t i = 0; i < frontier.vertices.size(); ++i) {
                    m_manual_object_->position(frontier.vertices[i]);
                    if (has_normals) { m_manual_object_->normal(normals[i]); }
                    m_manual_object_->colour(color);
                }
            } else {
                for (size_t i = 0; i < frontier.vertices.size(); ++i) {
                    m_manual_object_->position(frontier.vertices[i]);
                    m_manual_object_->colour(color);
                }
            }

            for (size_t i = 0; i < frontier.indices.size(); ++i) {
                m_manual_object_->index(frontier.indices[i]);
            }

            m_manual_object_->end();
        }
    }

    Ogre::ColourValue
    FrontierDisplay::ColorForFrontier(size_t index, const FrontierData &frontier) const {
        int mode = m_color_mode_property_->getOptionInt();

        if (mode == UNIFORM) {
            QColor qc = m_default_color_property_->getColor();
            return Ogre::ColourValue(
                static_cast<float>(qc.redF()),
                static_cast<float>(qc.greenF()),
                static_cast<float>(qc.blueF()),
                1.0f);
        }

        // generate a distinct hue from an integer key
        uint32_t key = (mode == BY_ID) ? frontier.id : static_cast<uint32_t>(index);
        // golden-ratio hue spread
        float hue = std::fmod(static_cast<float>(key) * 0.618033988749895f, 1.0f);
        QColor qc = QColor::fromHsvF(hue, 0.9, 0.95);
        return Ogre::ColourValue(
            static_cast<float>(qc.redF()),
            static_cast<float>(qc.greenF()),
            static_cast<float>(qc.blueF()),
            1.0f);
    }

    void
    FrontierDisplay::ComputeNormals(
        const std::vector<Ogre::Vector3> &vertices,
        const std::vector<uint32_t> &indices,
        std::vector<Ogre::Vector3> &normals) const {
        normals.assign(vertices.size(), Ogre::Vector3::ZERO);

        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            uint32_t i0 = indices[i];
            uint32_t i1 = indices[i + 1];
            uint32_t i2 = indices[i + 2];
            if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size()) {
                continue;
            }

            Ogre::Vector3 face_normal =
                (vertices[i1] - vertices[i0]).crossProduct(vertices[i2] - vertices[i0]);
            normals[i0] += face_normal;
            normals[i1] += face_normal;
            normals[i2] += face_normal;
        }

        for (auto &n: normals) {
            float len = n.length();
            if (len > 1e-6f) { n /= len; }
        }
    }

    void
    FrontierDisplay::Clear() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        if (m_manual_object_) { m_manual_object_->clear(); }
        m_frontiers_.clear();
        m_dim_ = 0;
        m_dirty_ = false;
    }

    bool
    FrontierDisplay::UpdateFromTf() {
        Ogre::Vector3 pos;
        Ogre::Quaternion orient;
        if (!context_->getFrameManager()->getTransform(m_header_, pos, orient)) { return false; }

        scene_node_->setOrientation(orient);
        scene_node_->setPosition(pos);
        return true;
    }
}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::FrontierDisplay, rviz_common::Display)
