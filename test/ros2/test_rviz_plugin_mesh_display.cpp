#include <erl_geometry_msgs/msg/mesh_msg.hpp>
#include <open3d/io/TriangleMeshIO.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>

class Node : public rclcpp::Node {

    rclcpp::Publisher<erl_geometry_msgs::msg::MeshMsg>::SharedPtr m_pub_3d_;
    rclcpp::Publisher<erl_geometry_msgs::msg::MeshMsg>::SharedPtr m_pub_2d_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    erl_geometry_msgs::msg::MeshMsg m_msg_3d_;
    erl_geometry_msgs::msg::MeshMsg m_msg_2d_;

public:
    Node()
        : rclcpp::Node("test_rviz_plugin_mesh_display") {
        this->declare_parameter("mesh_file", "data/replica-office-0.ply");
        this->declare_parameter("frame_id", "map");
        std::string mesh_file = this->get_parameter("mesh_file").as_string();
        std::string frame_id = this->get_parameter("frame_id").as_string();

        m_pub_3d_ = this->create_publisher<erl_geometry_msgs::msg::MeshMsg>("mesh", 1);
        m_pub_2d_ = this->create_publisher<erl_geometry_msgs::msg::MeshMsg>("mesh_2d", 1);

        auto mesh = open3d::io::CreateMeshFromFile(mesh_file);
        if (!mesh) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load mesh file: %s", mesh_file.c_str());
            rclcpp::shutdown();
            return;
        }

        // --- 3D mesh message (triangles) ---
        m_msg_3d_.header.frame_id = frame_id;
        m_msg_3d_.dim = 3;

        // populate vertices
        m_msg_3d_.mesh.vertices.resize(mesh->vertices_.size());
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            m_msg_3d_.mesh.vertices[i].x = mesh->vertices_[i][0];
            m_msg_3d_.mesh.vertices[i].y = mesh->vertices_[i][1];
            m_msg_3d_.mesh.vertices[i].z = mesh->vertices_[i][2];
        }

        // populate triangles
        m_msg_3d_.mesh.triangles.resize(mesh->triangles_.size());
        for (size_t i = 0; i < mesh->triangles_.size(); ++i) {
            m_msg_3d_.mesh.triangles[i].vertex_indices[0] = mesh->triangles_[i][0];
            m_msg_3d_.mesh.triangles[i].vertex_indices[1] = mesh->triangles_[i][1];
            m_msg_3d_.mesh.triangles[i].vertex_indices[2] = mesh->triangles_[i][2];
        }

        // populate vertex colors if available
        if (mesh->HasVertexColors()) {
            m_msg_3d_.vertex_colors.resize(mesh->vertex_colors_.size());
            for (size_t i = 0; i < mesh->vertex_colors_.size(); ++i) {
                m_msg_3d_.vertex_colors[i].r = static_cast<float>(mesh->vertex_colors_[i][0]);
                m_msg_3d_.vertex_colors[i].g = static_cast<float>(mesh->vertex_colors_[i][1]);
                m_msg_3d_.vertex_colors[i].b = static_cast<float>(mesh->vertex_colors_[i][2]);
                m_msg_3d_.vertex_colors[i].a = 1.0f;
            }
        }

        // --- 2D mesh message (line segments from triangle edges) ---
        m_msg_2d_.header.frame_id = frame_id;
        m_msg_2d_.dim = 2;

        // collect unique edges from triangles
        struct Edge {
            uint32_t a, b;
        };
        std::vector<Edge> edges;
        edges.reserve(mesh->triangles_.size() * 3);
        for (size_t i = 0; i < mesh->triangles_.size(); ++i) {
            uint32_t v0 = static_cast<uint32_t>(mesh->triangles_[i][0]);
            uint32_t v1 = static_cast<uint32_t>(mesh->triangles_[i][1]);
            uint32_t v2 = static_cast<uint32_t>(mesh->triangles_[i][2]);
            edges.push_back({std::min(v0, v1), std::max(v0, v1)});
            edges.push_back({std::min(v1, v2), std::max(v1, v2)});
            edges.push_back({std::min(v0, v2), std::max(v0, v2)});
        }
        std::sort(edges.begin(), edges.end(), [](const Edge &a, const Edge &b) {
            return a.a < b.a || (a.a == b.a && a.b < b.b);
        });
        edges.erase(
            std::unique(
                edges.begin(),
                edges.end(),
                [](const Edge &a, const Edge &b) { return a.a == b.a && a.b == b.b; }),
            edges.end());

        // use the same vertices (project to z=0 for 2D)
        m_msg_2d_.mesh.vertices.resize(mesh->vertices_.size());
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            m_msg_2d_.mesh.vertices[i].x = mesh->vertices_[i][0];
            m_msg_2d_.mesh.vertices[i].y = mesh->vertices_[i][1];
            m_msg_2d_.mesh.vertices[i].z = 0.0;
        }

        // store edges as line segments in mesh.triangles (only vertex_indices[0] and [1] used)
        m_msg_2d_.mesh.triangles.resize(edges.size());
        for (size_t i = 0; i < edges.size(); ++i) {
            m_msg_2d_.mesh.triangles[i].vertex_indices[0] = edges[i].a;
            m_msg_2d_.mesh.triangles[i].vertex_indices[1] = edges[i].b;
            m_msg_2d_.mesh.triangles[i].vertex_indices[2] = 0;  // unused
        }

        // copy vertex colors if available
        if (mesh->HasVertexColors()) {
            m_msg_2d_.vertex_colors.resize(mesh->vertex_colors_.size());
            for (size_t i = 0; i < mesh->vertex_colors_.size(); ++i) {
                m_msg_2d_.vertex_colors[i].r = static_cast<float>(mesh->vertex_colors_[i][0]);
                m_msg_2d_.vertex_colors[i].g = static_cast<float>(mesh->vertex_colors_[i][1]);
                m_msg_2d_.vertex_colors[i].b = static_cast<float>(mesh->vertex_colors_[i][2]);
                m_msg_2d_.vertex_colors[i].a = 1.0f;
            }
        }

        // publish both
        m_msg_3d_.header.stamp = this->now();
        m_pub_3d_->publish(m_msg_3d_);
        m_msg_2d_.header.stamp = this->now();
        m_pub_2d_->publish(m_msg_2d_);

        m_timer_ =
            this->create_wall_timer(std::chrono::seconds(1), std::bind(&Node::Callback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "Published 3D mesh (%zu vertices, %zu triangles) and 2D mesh (%zu vertices, %zu edges)",
            mesh->vertices_.size(),
            mesh->triangles_.size(),
            m_msg_2d_.mesh.vertices.size(),
            edges.size());
    }

private:
    void
    Callback() {
        auto stamp = this->now();
        m_msg_3d_.header.stamp = stamp;
        m_pub_3d_->publish(m_msg_3d_);
        m_msg_2d_.header.stamp = stamp;
        m_pub_2d_->publish(m_msg_2d_);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
