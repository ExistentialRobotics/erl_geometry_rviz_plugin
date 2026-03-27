#include <open3d/io/TriangleMeshIO.h>
#include <ros/ros.h>
#include <erl_geometry_msgs/MeshMsg.h>

#include <filesystem>

class Node {

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Timer m_timer_;
    erl_geometry_msgs::MeshMsg m_msg_;

public:
    Node(ros::NodeHandle& nh)
        : m_nh_(nh) {
        std::string mesh_file = "data/bunny_z_up.ply";
        if (!m_nh_.getParam("mesh_file", mesh_file)) {
            ROS_FATAL("Parameter 'mesh_file' not set");
            ros::shutdown();
            return;
        }
        if (!std::filesystem::exists(mesh_file)) {
            ROS_FATAL("Mesh file does not exist: %s", mesh_file.c_str());
            ros::shutdown();
            return;
        }

        std::string frame_id = "map";
        m_nh_.param<std::string>("frame_id", frame_id, "map");

        m_pub_ = m_nh_.advertise<erl_geometry_msgs::MeshMsg>("mesh", 1, true);

        auto mesh = open3d::io::CreateMeshFromFile(mesh_file);
        if (!mesh) {
            ROS_FATAL("Failed to load mesh file: %s", mesh_file.c_str());
            ros::shutdown();
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

        m_msg_.header.stamp = ros::Time::now();
        m_pub_.publish(m_msg_);
        m_timer_ = m_nh_.createTimer(ros::Duration(1.0), &Node::Callback, this);
        ROS_INFO(
            "Published mesh with %zu vertices and %zu triangles",
            mesh->vertices_.size(),
            mesh->triangles_.size());
    }

private:
    void
    Callback(const ros::TimerEvent& /* event */) {
        m_msg_.header.stamp = ros::Time::now();
        m_pub_.publish(m_msg_);
    }
};

int
main(int argc, char** argv) {
    ros::init(argc, argv, "test_rviz_plugin_mesh_display");
    ros::NodeHandle nh("~");
    Node node(nh);
    ros::spin();
    return 0;
}
