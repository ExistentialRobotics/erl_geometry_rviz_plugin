#include <open3d/io/TriangleMeshIO.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <filesystem>

class Node : public rclcpp::Node {

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    sensor_msgs::msg::PointCloud2 m_msg_;

public:
    Node()
        : rclcpp::Node("test_rviz_plugin_better_point_cloud2_display") {
        // Declare parameter
        this->declare_parameter("mesh_file", "data/bunny_z_up.ply");

        // Initialize the node
        std::string mesh_file = this->get_parameter("mesh_file").as_string();

        m_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);

        auto mesh = open3d::io::CreateMeshFromFile(mesh_file);
        if (!mesh) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load mesh file: %s", mesh_file.c_str());
            rclcpp::shutdown();
            return;
        }

        mesh->ComputeVertexNormals();
        m_msg_.header.frame_id = "map";
        m_msg_.height = 1;
        m_msg_.width = mesh->vertices_.size();
        m_msg_.fields.resize(6);
        m_msg_.fields[0].name = "x";
        m_msg_.fields[0].offset = 0;
        m_msg_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[0].count = 1;  // number of elements per point
        m_msg_.fields[1].name = "y";
        m_msg_.fields[1].offset = 4;
        m_msg_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[1].count = 1;
        m_msg_.fields[2].name = "z";
        m_msg_.fields[2].offset = 8;
        m_msg_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[2].count = 1;
        m_msg_.fields[3].name = "normal_x";
        m_msg_.fields[3].offset = 12;
        m_msg_.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[3].count = 1;
        m_msg_.fields[4].name = "normal_y";
        m_msg_.fields[4].offset = 16;
        m_msg_.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[4].count = 1;
        m_msg_.fields[5].name = "normal_z";
        m_msg_.fields[5].offset = 20;
        m_msg_.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_msg_.fields[5].count = 1;
        m_msg_.is_bigendian = false;
        m_msg_.point_step = 24;  // size of one point in bytes
        m_msg_.row_step = m_msg_.point_step * m_msg_.width;
        m_msg_.data.resize(m_msg_.row_step);
        m_msg_.is_dense = true;
        char* data_ptr = reinterpret_cast<char*>(m_msg_.data.data());
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            const Eigen::Vector3f vertex = mesh->vertices_[i].cast<float>();
            const Eigen::Vector3f normal = mesh->vertex_normals_[i].cast<float>();
            std::memcpy(data_ptr, &vertex[0], sizeof(float) * 3);
            std::memcpy(data_ptr + 12, &normal[0], sizeof(float) * 3);
            data_ptr += m_msg_.point_step;
        }
        m_msg_.header.stamp = this->now();
        m_pub_->publish(m_msg_);
        m_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Node::Callback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "Published point cloud with %zu points",
            mesh->vertices_.size());
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
