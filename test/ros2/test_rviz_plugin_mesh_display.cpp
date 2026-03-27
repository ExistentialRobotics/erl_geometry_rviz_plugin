#include <open3d/io/TriangleMeshIO.h>
#include <rclcpp/rclcpp.hpp>
#include <erl_geometry_msgs/msg/mesh_msg.hpp>

#include <filesystem>

class Node : public rclcpp::Node {

    rclcpp::Publisher<erl_geometry_msgs::msg::MeshMsg>::SharedPtr m_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    erl_geometry_msgs::msg::MeshMsg m_msg_;

public:
    Node()
        : rclcpp::Node("test_rviz_plugin_mesh_display") {
        this->declare_parameter("mesh_file", "data/replica-office-0.ply");
        this->declare_parameter("frame_id", "map");
        std::string mesh_file = this->get_parameter("mesh_file").as_string();
        std::string frame_id = this->get_parameter("frame_id").as_string();

        m_pub_ = this->create_publisher<erl_geometry_msgs::msg::MeshMsg>("mesh", 1);

        auto mesh = open3d::io::CreateMeshFromFile(mesh_file);
        if (!mesh) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load mesh file: %s", mesh_file.c_str());
            rclcpp::shutdown();
            return;
        }

        // header
        m_msg_.header.frame_id = frame_id;

        // populate vertices
        m_msg_.mesh.vertices.resize(mesh->vertices_.size());
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            m_msg_.mesh.vertices[i].x = mesh->vertices_[i][0];
            m_msg_.mesh.vertices[i].y = mesh->vertices_[i][1];
            m_msg_.mesh.vertices[i].z = mesh->vertices_[i][2];
        }

        // populate triangles
        m_msg_.mesh.triangles.resize(mesh->triangles_.size());
        for (size_t i = 0; i < mesh->triangles_.size(); ++i) {
            m_msg_.mesh.triangles[i].vertex_indices[0] = mesh->triangles_[i][0];
            m_msg_.mesh.triangles[i].vertex_indices[1] = mesh->triangles_[i][1];
            m_msg_.mesh.triangles[i].vertex_indices[2] = mesh->triangles_[i][2];
        }

        // populate vertex colors if available
        if (mesh->HasVertexColors()) {
            m_msg_.vertex_colors.resize(mesh->vertex_colors_.size());
            for (size_t i = 0; i < mesh->vertex_colors_.size(); ++i) {
                m_msg_.vertex_colors[i].r = static_cast<float>(mesh->vertex_colors_[i][0]);
                m_msg_.vertex_colors[i].g = static_cast<float>(mesh->vertex_colors_[i][1]);
                m_msg_.vertex_colors[i].b = static_cast<float>(mesh->vertex_colors_[i][2]);
                m_msg_.vertex_colors[i].a = 1.0f;
            }
        }

        m_msg_.header.stamp = this->now();
        m_pub_->publish(m_msg_);
        m_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Node::Callback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "Published mesh with %zu vertices and %zu triangles",
            mesh->vertices_.size(),
            mesh->triangles_.size());
    }

private:
    void
    Callback() {
        m_msg_.header.stamp = this->now();
        m_pub_->publish(m_msg_);
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
