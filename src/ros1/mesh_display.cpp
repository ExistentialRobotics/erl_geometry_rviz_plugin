#include "erl_geometry_rviz_plugin/ros1/mesh_display.hpp"

#include <boost/bind.hpp>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/status_property.h>
#include <rviz/visualization_manager.h>

#include <fstream>
#include <QObject>
#include <sstream>

namespace erl::geometry::rviz_plugin {

    MeshDisplay::MeshDisplay()
        : rviz::Display(), m_queue_size_(5), m_messages_received_(0) {

        m_queue_size_property_ = new rviz::IntProperty(
            "Queue Size",
            m_queue_size_,
            "Size of the incoming message queue.",
            this,
            SLOT(UpdateQueueSize()));
        m_queue_size_property_->setMin(1);

        m_topic_property_ = new rviz::RosTopicProperty(
            "Mesh Topic",
            "",
            QString::fromStdString(ros::message_traits::datatype<erl_geometry_msgs::MeshMsg>()),
            "erl_geometry_msgs::MeshMsg topic to subscribe to.",
            this,
            SLOT(UpdateTopic()));

        m_ply_file_property_ = new rviz::StringProperty(
            "PLY File",
            "",
            "Path to a .ply file to load. Leave empty to use the topic.",
            this,
            SLOT(UpdatePlyFile()));

        m_default_color_property_ = new rviz::ColorProperty(
            "Default Color",
            QColor(180, 180, 180),
            "Default color when the mesh has no per-vertex colors.",
            this,
            SLOT(UpdateDefaultColor()));

        m_alpha_property_ =
            new rviz::FloatProperty("Alpha", 1.0, "Mesh transparency.", this, SLOT(UpdateAlpha()));
        m_alpha_property_->setMin(0.0);
        m_alpha_property_->setMax(1.0);

        m_use_normals_property_ = new rviz::BoolProperty(
            "Compute Normals",
            true,
            "Compute face normals for lighting.",
            this,
            SLOT(UpdateUseNormals()));

        m_backface_culling_property_ = new rviz::BoolProperty(
            "Backface Culling",
            false,
            "Cull back faces. When off, both sides of triangles are rendered.",
            this,
            SLOT(UpdateBackfaceCulling()));
    }

    MeshDisplay::~MeshDisplay() {
        Unsubscribe();
        Clear();
        if (m_manual_object_) {
            scene_manager_->destroyManualObject(m_manual_object_);
            m_manual_object_ = nullptr;
        }
        if (!m_material_.isNull()) {
            Ogre::ResourcePtr res_ptr(m_material_);
            Ogre::MaterialManager::getSingleton().remove(res_ptr);
            m_material_.setNull();
        }
    }

    void
    MeshDisplay::onInitialize() {
        static int material_counter = 0;
        std::string material_name = "MeshDisplayMaterial_" + std::to_string(material_counter++);
        m_material_ = Ogre::MaterialManager::getSingleton().create(
            material_name,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        m_material_->setReceiveShadows(false);
        m_material_->getTechnique(0)->setLightingEnabled(true);
        m_material_->getTechnique(0)->getPass(0)->setVertexColourTracking(
            Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
        m_material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
        m_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        m_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);

        m_manual_object_ = scene_manager_->createManualObject();
        scene_node_->attachObject(m_manual_object_);
    }

    void
    MeshDisplay::update(float /* wall_dt */, float /* ros_dt */) {
        if (m_mesh_dirty_) {
            std::lock_guard<std::mutex> lock(m_mutex_);
            BuildMesh();
            m_mesh_dirty_ = false;
        }
        UpdateFromTf();
    }

    void
    MeshDisplay::reset() {
        Clear();
        m_messages_received_ = 0;
        setStatus(rviz::StatusProperty::Ok, "Messages", QString("0 mesh messages received"));
    }

    void
    MeshDisplay::UpdateQueueSize() {
        m_queue_size_ = m_queue_size_property_->getInt();
        Subscribe();
    }

    void
    MeshDisplay::UpdateTopic() {
        Unsubscribe();
        reset();
        Subscribe();
        context_->queueRender();
    }

    void
    MeshDisplay::UpdatePlyFile() {
        std::string file_path = m_ply_file_property_->getStdString();
        if (file_path.empty()) { return; }
        LoadPlyFile(file_path);
    }

    void
    MeshDisplay::UpdateAlpha() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_mesh_dirty_ = true;
    }

    void
    MeshDisplay::UpdateDefaultColor() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_mesh_dirty_ = true;
    }

    void
    MeshDisplay::UpdateUseNormals() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        if (m_use_normals_property_->getBool() && !m_vertices_.empty()) {
            ComputeNormals();
        } else {
            m_normals_.clear();
        }
        m_mesh_dirty_ = true;
    }

    void
    MeshDisplay::UpdateBackfaceCulling() {
        if (m_material_.isNull()) { return; }
        m_material_->getTechnique(0)->getPass(0)->setCullingMode(
            m_backface_culling_property_->getBool() ? Ogre::CULL_CLOCKWISE : Ogre::CULL_NONE);
        context_->queueRender();
    }

    void
    MeshDisplay::onEnable() {
        scene_node_->setVisible(true);
        Subscribe();
    }

    void
    MeshDisplay::onDisable() {
        scene_node_->setVisible(false);
        Unsubscribe();
        Clear();
    }

    void
    MeshDisplay::Subscribe() {
        if (!isEnabled()) { return; }

        try {
            Unsubscribe();
            const std::string &topic = m_topic_property_->getStdString();
            if (!topic.empty()) {
                m_sub_ =
                    std::make_shared<message_filters::Subscriber<erl_geometry_msgs::MeshMsg>>();
                m_sub_->subscribe(threaded_nh_, topic, m_queue_size_);
                m_sub_->registerCallback(
                    boost::bind(&MeshDisplay::IncomingMessageCallback, this, _1));
            }
        } catch (ros::Exception &e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error subscribing: ") + e.what()).c_str());
        }
    }

    void
    MeshDisplay::Unsubscribe() {
        try {
            m_sub_.reset();
        } catch (ros::Exception &e) {
            setStatus(
                rviz::StatusProperty::Error,
                "Topic",
                (std::string("Error unsubscribing: ") + e.what()).c_str());
        }
    }

    void
    MeshDisplay::IncomingMessageCallback(const erl_geometry_msgs::MeshMsg::ConstPtr &msg) {
        ++m_messages_received_;
        setStatus(
            rviz::StatusProperty::Ok,
            "Messages",
            QString::number(m_messages_received_) + " mesh messages received");

        std::lock_guard<std::mutex> lock(m_mutex_);

        m_header_ = msg->header;
        m_dim_ = msg->dim;
        const auto &mesh = msg->mesh;
        const int indices_per_face = (m_dim_ == 2) ? 2 : 3;

        // extract vertices
        m_vertices_.resize(mesh.vertices.size());
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            m_vertices_[i] = Ogre::Vector3(
                static_cast<float>(mesh.vertices[i].x),
                static_cast<float>(mesh.vertices[i].y),
                static_cast<float>(mesh.vertices[i].z));
        }

        // extract indices (3 per triangle for 3D, 2 per line segment for 2D)
        m_indices_.resize(mesh.triangles.size() * indices_per_face);
        for (size_t i = 0; i < mesh.triangles.size(); ++i) {
            for (int k = 0; k < indices_per_face; ++k) {
                m_indices_[i * indices_per_face + k] = mesh.triangles[i].vertex_indices[k];
            }
        }

        // extract colors: prefer vertex_colors, then expand face_colors to per-vertex
        m_colors_.clear();
        if (msg->vertex_colors.size() == mesh.vertices.size()) {
            m_colors_.resize(mesh.vertices.size());
            for (size_t i = 0; i < msg->vertex_colors.size(); ++i) {
                m_colors_[i] = Ogre::ColourValue(
                    msg->vertex_colors[i].r,
                    msg->vertex_colors[i].g,
                    msg->vertex_colors[i].b,
                    msg->vertex_colors[i].a);
            }
        } else if (msg->face_colors.size() == mesh.triangles.size()) {
            m_colors_.assign(mesh.vertices.size(), Ogre::ColourValue::ZERO);
            std::vector<int> counts(mesh.vertices.size(), 0);
            for (size_t f = 0; f < mesh.triangles.size(); ++f) {
                Ogre::ColourValue fc(
                    msg->face_colors[f].r,
                    msg->face_colors[f].g,
                    msg->face_colors[f].b,
                    msg->face_colors[f].a);
                for (int k = 0; k < indices_per_face; ++k) {
                    uint32_t vi = mesh.triangles[f].vertex_indices[k];
                    if (vi < mesh.vertices.size()) {
                        m_colors_[vi] = m_colors_[vi] + fc;
                        counts[vi]++;
                    }
                }
            }
            for (size_t i = 0; i < m_colors_.size(); ++i) {
                if (counts[i] > 0) { m_colors_[i] = m_colors_[i] / static_cast<float>(counts[i]); }
            }
        }

        if (m_dim_ == 3 && m_use_normals_property_->getBool()) {
            ComputeNormals();
        } else {
            m_normals_.clear();
        }

        m_mesh_dirty_ = true;
    }

    void
    MeshDisplay::LoadPlyFile(const std::string &file_path) {
        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
            setStatusStd(rviz::StatusProperty::Error, "PLY File", "Cannot open file: " + file_path);
            return;
        }

        // parse PLY header
        std::string line;
        int num_vertices = 0;
        int num_faces = 0;
        bool has_colors = false;
        bool has_alpha = false;
        bool is_binary_le = false;
        bool is_binary_be = false;
        bool in_vertex_element = false;

        // track vertex property order
        struct VertexProp {
            std::string name;
            std::string type;
        };

        std::vector<VertexProp> vertex_props;

        std::getline(file, line);
        if (line.substr(0, 3) != "ply") {
            setStatusStd(rviz::StatusProperty::Error, "PLY File", "Not a valid PLY file.");
            return;
        }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            iss >> token;

            if (token == "format") {
                std::string fmt;
                iss >> fmt;
                if (fmt == "binary_little_endian") {
                    is_binary_le = true;
                } else if (fmt == "binary_big_endian") {
                    is_binary_be = true;
                }
            } else if (token == "element") {
                std::string elem_type;
                iss >> elem_type;
                if (elem_type == "vertex") {
                    iss >> num_vertices;
                    in_vertex_element = true;
                } else {
                    if (elem_type == "face") { iss >> num_faces; }
                    in_vertex_element = false;
                }
            } else if (token == "property" && in_vertex_element) {
                std::string type, name;
                iss >> type >> name;
                if (type != "list") {
                    vertex_props.push_back({name, type});
                    if (name == "red" || name == "r") { has_colors = true; }
                    if (name == "alpha" || name == "a") { has_alpha = true; }
                }
            } else if (token == "end_header") {
                break;
            }
        }

        if (is_binary_be) {
            setStatusStd(
                rviz::StatusProperty::Error,
                "PLY File",
                "Binary big-endian PLY not supported.");
            return;
        }

        std::lock_guard<std::mutex> lock(m_mutex_);
        m_vertices_.resize(num_vertices);
        m_colors_.clear();
        if (has_colors) { m_colors_.resize(num_vertices); }

        // find property indices
        int x_idx = -1, y_idx = -1, z_idx = -1;
        int r_idx = -1, g_idx = -1, b_idx = -1, a_idx = -1;
        for (int i = 0; i < static_cast<int>(vertex_props.size()); ++i) {
            const auto &name = vertex_props[i].name;
            if (name == "x") x_idx = i;
            else if (name == "y")
                y_idx = i;
            else if (name == "z")
                z_idx = i;
            else if (name == "red" || name == "r")
                r_idx = i;
            else if (name == "green" || name == "g")
                g_idx = i;
            else if (name == "blue" || name == "b")
                b_idx = i;
            else if (name == "alpha" || name == "a")
                a_idx = i;
        }

        // helper to get byte size of a PLY type
        auto ply_type_size = [](const std::string &t) -> int {
            if (t == "char" || t == "int8" || t == "uchar" || t == "uint8") return 1;
            if (t == "short" || t == "int16" || t == "ushort" || t == "uint16") return 2;
            if (t == "int" || t == "int32" || t == "uint" || t == "uint32" || t == "float" ||
                t == "float32")
                return 4;
            if (t == "double" || t == "float64") return 8;
            return 0;
        };

        if (is_binary_le) {
            // compute vertex stride
            int vertex_stride = 0;
            std::vector<int> prop_offsets(vertex_props.size());
            for (size_t i = 0; i < vertex_props.size(); ++i) {
                prop_offsets[i] = vertex_stride;
                vertex_stride += ply_type_size(vertex_props[i].type);
            }

            std::vector<char> vertex_buf(static_cast<size_t>(vertex_stride));
            for (int v = 0; v < num_vertices; ++v) {
                file.read(vertex_buf.data(), vertex_stride);

                auto read_float = [&](int idx) -> float {
                    const auto &type = vertex_props[idx].type;
                    const char *ptr = vertex_buf.data() + prop_offsets[idx];
                    if (type == "float" || type == "float32") {
                        float val;
                        std::memcpy(&val, ptr, sizeof(float));
                        return val;
                    }
                    if (type == "double" || type == "float64") {
                        double val;
                        std::memcpy(&val, ptr, sizeof(double));
                        return static_cast<float>(val);
                    }
                    return 0.0f;
                };

                auto read_uchar = [&](int idx) -> uint8_t {
                    return static_cast<uint8_t>(vertex_buf[prop_offsets[idx]]);
                };

                m_vertices_[v] =
                    Ogre::Vector3(read_float(x_idx), read_float(y_idx), read_float(z_idx));

                if (has_colors && r_idx >= 0 && g_idx >= 0 && b_idx >= 0) {
                    float r = static_cast<float>(read_uchar(r_idx)) / 255.0f;
                    float g = static_cast<float>(read_uchar(g_idx)) / 255.0f;
                    float b = static_cast<float>(read_uchar(b_idx)) / 255.0f;
                    float a = (has_alpha && a_idx >= 0)
                                  ? static_cast<float>(read_uchar(a_idx)) / 255.0f
                                  : 1.0f;
                    m_colors_[v] = Ogre::ColourValue(r, g, b, a);
                }
            }

            // read faces
            m_indices_.clear();
            m_indices_.reserve(static_cast<size_t>(num_faces) * 3);
            for (int f = 0; f < num_faces; ++f) {
                uint8_t count;
                file.read(reinterpret_cast<char *>(&count), 1);
                std::vector<uint32_t> face_indices(count);
                if (count <= 4) {
                    // try reading as int32
                    file.read(
                        reinterpret_cast<char *>(face_indices.data()),
                        static_cast<std::streamsize>(count * sizeof(uint32_t)));
                } else {
                    for (uint8_t i = 0; i < count; ++i) {
                        uint32_t idx;
                        file.read(reinterpret_cast<char *>(&idx), sizeof(uint32_t));
                        face_indices[i] = idx;
                    }
                }
                // triangulate (fan)
                for (uint8_t i = 1; i + 1 < count; ++i) {
                    m_indices_.push_back(face_indices[0]);
                    m_indices_.push_back(face_indices[i]);
                    m_indices_.push_back(face_indices[i + 1]);
                }
            }
        } else {
            // ASCII format
            for (int v = 0; v < num_vertices; ++v) {
                std::vector<double> values(vertex_props.size());
                for (size_t i = 0; i < vertex_props.size(); ++i) { file >> values[i]; }

                m_vertices_[v] = Ogre::Vector3(
                    static_cast<float>(values[x_idx]),
                    static_cast<float>(values[y_idx]),
                    static_cast<float>(values[z_idx]));

                if (has_colors && r_idx >= 0 && g_idx >= 0 && b_idx >= 0) {
                    float r = static_cast<float>(values[r_idx]) / 255.0f;
                    float g = static_cast<float>(values[g_idx]) / 255.0f;
                    float b = static_cast<float>(values[b_idx]) / 255.0f;
                    float a = (has_alpha && a_idx >= 0) ? static_cast<float>(values[a_idx]) / 255.0f
                                                        : 1.0f;
                    m_colors_[v] = Ogre::ColourValue(r, g, b, a);
                }
            }

            // read faces
            m_indices_.clear();
            m_indices_.reserve(static_cast<size_t>(num_faces) * 3);
            for (int f = 0; f < num_faces; ++f) {
                int count;
                file >> count;
                std::vector<uint32_t> face_indices(count);
                for (int i = 0; i < count; ++i) { file >> face_indices[i]; }
                // triangulate (fan)
                for (int i = 1; i + 1 < count; ++i) {
                    m_indices_.push_back(face_indices[0]);
                    m_indices_.push_back(face_indices[i]);
                    m_indices_.push_back(face_indices[i + 1]);
                }
            }
        }

        if (m_use_normals_property_->getBool()) {
            ComputeNormals();
        } else {
            m_normals_.clear();
        }

        m_mesh_dirty_ = true;
        setStatusStd(
            rviz::StatusProperty::Ok,
            "PLY File",
            "Loaded " + std::to_string(num_vertices) + " vertices, " + std::to_string(num_faces) +
                " faces from " + file_path);
    }

    void
    MeshDisplay::BuildMesh() {
        if (!m_manual_object_) { return; }

        m_manual_object_->clear();
        if (m_vertices_.empty() || m_indices_.empty()) { return; }

        float alpha = m_alpha_property_->getFloat();
        QColor default_color = m_default_color_property_->getColor();
        Ogre::ColourValue default_ogre_color(
            default_color.redF(),
            default_color.greenF(),
            default_color.blueF(),
            alpha);

        bool has_per_vertex_normals = m_normals_.size() == m_vertices_.size();
        bool has_per_vertex_colors = m_colors_.size() == m_vertices_.size();

        // update material alpha
        m_material_->getTechnique(0)->getPass(0)->setSceneBlending(
            alpha < 1.0f ? Ogre::SBT_TRANSPARENT_ALPHA : Ogre::SBT_REPLACE);
        m_material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(alpha >= 1.0f);
        m_material_->getTechnique(0)->setLightingEnabled(has_per_vertex_normals);

        auto render_op = (m_dim_ == 2) ? Ogre::RenderOperation::OT_LINE_LIST
                                       : Ogre::RenderOperation::OT_TRIANGLE_LIST;

        m_manual_object_->estimateVertexCount(static_cast<uint32_t>(m_vertices_.size()));
        m_manual_object_->estimateIndexCount(static_cast<uint32_t>(m_indices_.size()));
        m_manual_object_->begin(m_material_->getName(), render_op);

        for (size_t i = 0; i < m_vertices_.size(); ++i) {
            m_manual_object_->position(m_vertices_[i]);
            if (has_per_vertex_normals) { m_manual_object_->normal(m_normals_[i]); }
            Ogre::ColourValue color = has_per_vertex_colors ? m_colors_[i] : default_ogre_color;
            color.a = alpha;
            m_manual_object_->colour(color);
        }

        for (size_t i = 0; i < m_indices_.size(); ++i) { m_manual_object_->index(m_indices_[i]); }

        m_manual_object_->end();
    }

    void
    MeshDisplay::ComputeNormals() {
        m_normals_.assign(m_vertices_.size(), Ogre::Vector3::ZERO);

        for (size_t i = 0; i + 2 < m_indices_.size(); i += 3) {
            uint32_t i0 = m_indices_[i];
            uint32_t i1 = m_indices_[i + 1];
            uint32_t i2 = m_indices_[i + 2];
            if (i0 >= m_vertices_.size() || i1 >= m_vertices_.size() || i2 >= m_vertices_.size()) {
                continue;
            }

            Ogre::Vector3 v0 = m_vertices_[i0];
            Ogre::Vector3 v1 = m_vertices_[i1];
            Ogre::Vector3 v2 = m_vertices_[i2];

            Ogre::Vector3 face_normal = (v1 - v0).crossProduct(v2 - v0);
            // accumulate (area-weighted)
            m_normals_[i0] += face_normal;
            m_normals_[i1] += face_normal;
            m_normals_[i2] += face_normal;
        }

        for (auto &n: m_normals_) {
            float len = n.length();
            if (len > 1e-6f) { n /= len; }
        }
    }

    void
    MeshDisplay::Clear() {
        std::lock_guard<std::mutex> lock(m_mutex_);
        if (m_manual_object_) { m_manual_object_->clear(); }
        m_dim_ = 3;
        m_vertices_.clear();
        m_indices_.clear();
        m_normals_.clear();
        m_colors_.clear();
        m_mesh_dirty_ = false;
    }

    bool
    MeshDisplay::UpdateFromTf() {
        Ogre::Vector3 pos;
        Ogre::Quaternion orient;
        if (!context_->getFrameManager()->getTransform(m_header_, pos, orient)) { return false; }

        scene_node_->setOrientation(orient);
        scene_node_->setPosition(pos);
        return true;
    }
}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::MeshDisplay, rviz::Display)
