#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry_msgs/ros1/occupancy_tree_msg.hpp"

#include <erl_geometry_msgs/FrontierArray.h>
#include <erl_geometry_msgs/OccupancyTreeMsg.h>
#include <ros/ros.h>

using Dtype = double;
using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;
using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;
using Vector2 = Eigen::Vector2<Dtype>;
using Vector3 = Eigen::Vector3<Dtype>;
using Matrix2X = Eigen::Matrix2X<Dtype>;
using Matrix3X = Eigen::Matrix3X<Dtype>;
using VectorX = Eigen::VectorX<Dtype>;

/// Build a quadtree with a circular scan: free interior, occupied ring, unknown exterior.
static std::shared_ptr<OccupancyQuadtree>
BuildCircleTree(const Dtype resolution = 0.04) {
    auto setting = std::make_shared<OccupancyQuadtree::Setting>();
    setting->resolution = static_cast<float>(resolution);
    auto tree = std::make_shared<OccupancyQuadtree>(setting);

    constexpr long n = 90;
    VectorX angles = VectorX::LinSpaced(n, -M_PI, M_PI);
    Matrix2X points(2, n);
    const Vector2 sensor_origin(0., 0.);

    for (long i = 0; i < n; ++i) {
        constexpr Dtype radius = 1.0;
        points.col(i) << std::cos(angles[i]) * radius, std::sin(angles[i]) * radius;
    }

    tree->InsertPointCloud(
        points,
        sensor_origin,
        /*min_range=*/0.0,
        /*max_range=*/-1.0,
        /*with_count=*/false,
        /*parallel=*/false,
        /*lazy_eval=*/false,
        /*discrete=*/false);

    return tree;
}

/// Build an octree with a spherical scan: free interior, occupied shell, unknown exterior.
static std::shared_ptr<OccupancyOctree>
BuildSphereTree(const Dtype resolution = 0.1) {
    auto setting = std::make_shared<OccupancyOctree::Setting>();
    setting->resolution = static_cast<float>(resolution);
    auto tree = std::make_shared<OccupancyOctree>(setting);

    constexpr long n = 1000;
    Matrix3X points(3, n);
    const Vector3 sensor_origin(0., 0., 0.);

    const Dtype golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
    for (long i = 0; i < n; ++i) {
        constexpr Dtype radius = 1.0;
        const Dtype theta = std::acos(1.0 - 2.0 * (static_cast<Dtype>(i) + 0.5) / n);
        const Dtype phi = 2.0 * M_PI * static_cast<Dtype>(i) / golden_ratio;
        points.col(i) << radius * std::sin(theta) * std::cos(phi),
            radius * std::sin(theta) * std::sin(phi), radius * std::cos(theta);
    }

    tree->InsertPointCloud(
        points,
        sensor_origin,
        /*min_range=*/0.0,
        /*max_range=*/-1.0,
        /*with_count=*/false,
        /*parallel=*/false,
        /*lazy_eval=*/false,
        /*discrete=*/false);

    return tree;
}

/// Convert 2D quadtree frontiers to a FrontierArray message (dim=2, line segments).
static erl_geometry_msgs::FrontierArray
QuadtreeFrontiersToMsg(
    const std::vector<OccupancyQuadtree::Frontier> &frontiers,
    Dtype resolution,
    const std::string &frame_id) {
    erl_geometry_msgs::FrontierArray msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.dim = 2;
    msg.resolution = resolution;

    for (uint32_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        erl_geometry_msgs::Frontier f_msg;
        f_msg.id = fi;
        f_msg.score = static_cast<double>(frontier.cols());

        // polyline vertices (z=0 for 2D)
        f_msg.vertices.resize(frontier.cols());
        for (Eigen::Index i = 0; i < frontier.cols(); ++i) {
            f_msg.vertices[i].x = frontier(0, i);
            f_msg.vertices[i].y = frontier(1, i);
            f_msg.vertices[i].z = 0.0;
        }

        // consecutive vertex pairs form line segments
        f_msg.indices.reserve((frontier.cols() - 1) * 2);
        for (Eigen::Index i = 0; i + 1 < frontier.cols(); ++i) {
            f_msg.indices.push_back(static_cast<uint32_t>(i));
            f_msg.indices.push_back(static_cast<uint32_t>(i + 1));
        }

        msg.frontiers.push_back(std::move(f_msg));
    }

    return msg;
}

/// Convert 3D octree frontiers to a FrontierArray message (dim=3, triangle faces).
static erl_geometry_msgs::FrontierArray
OctreeFrontiersToMsg(
    const std::vector<OccupancyOctree::Frontier> &frontiers,
    Dtype resolution,
    const std::string &frame_id) {
    erl_geometry_msgs::FrontierArray msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.dim = 3;
    msg.resolution = resolution;

    for (uint32_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        erl_geometry_msgs::Frontier f_msg;
        f_msg.id = fi;
        f_msg.score = static_cast<double>(frontier.faces.size());

        f_msg.vertices.resize(frontier.vertices.size());
        for (size_t i = 0; i < frontier.vertices.size(); ++i) {
            f_msg.vertices[i].x = frontier.vertices[i].x();
            f_msg.vertices[i].y = frontier.vertices[i].y();
            f_msg.vertices[i].z = frontier.vertices[i].z();
        }

        f_msg.indices.reserve(frontier.faces.size() * 3);
        for (const auto &face: frontier.faces) {
            f_msg.indices.push_back(static_cast<uint32_t>(face[0]));
            f_msg.indices.push_back(static_cast<uint32_t>(face[1]));
            f_msg.indices.push_back(static_cast<uint32_t>(face[2]));
        }

        msg.frontiers.push_back(std::move(f_msg));
    }

    return msg;
}

class TestFrontierDisplayNode {

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_frontier_;
    ros::Publisher m_pub_tree_;
    ros::Timer m_timer_;

    bool m_is_3d_ = true;
    bool m_published_ = false;

    // 2D data
    std::shared_ptr<OccupancyQuadtree> m_quadtree_;
    std::vector<OccupancyQuadtree::Frontier> m_frontiers_2d_;

    // 3D data
    std::shared_ptr<OccupancyOctree> m_octree_;
    std::vector<OccupancyOctree::Frontier> m_frontiers_3d_;

public:
    TestFrontierDisplayNode(ros::NodeHandle &nh)
        : m_nh_(nh) {

        m_nh_.param("is_3d", m_is_3d_, true);

        m_pub_frontier_ = m_nh_.advertise<erl_geometry_msgs::FrontierArray>("frontiers", 1, true);
        m_pub_tree_ = m_nh_.advertise<erl_geometry_msgs::OccupancyTreeMsg>("tree", 1, true);

        if (m_is_3d_) {
            ROS_INFO("Building sphere octree (3D)...");
            m_octree_ = BuildSphereTree(0.1);
            m_frontiers_3d_ = m_octree_->ExtractFrontiers();
            ROS_INFO("Extracted %zu 3D frontiers", m_frontiers_3d_.size());
        } else {
            ROS_INFO("Building circle quadtree (2D)...");
            m_quadtree_ = BuildCircleTree(0.04);
            m_frontiers_2d_ = m_quadtree_->ExtractFrontiers();
            ROS_INFO("Extracted %zu 2D frontiers", m_frontiers_2d_.size());
        }

        m_timer_ = m_nh_.createTimer(
            ros::Duration(1.0),
            &TestFrontierDisplayNode::Publish,
            this);
    }

private:
    void
    Publish(const ros::TimerEvent & /* event */) {
        if (!m_published_) {
            ROS_INFO("Publishing tree and frontiers...");
            m_published_ = true;
        }

        const std::string frame_id = "map";

        // publish occupancy tree
        {
            using namespace erl::geometry;
            erl_geometry_msgs::OccupancyTreeMsg tree_msg;
            bool ok = false;
            if (m_is_3d_) {
                ok = SaveToOccupancyTreeMsg<Dtype>(m_octree_, 1.0, true, tree_msg);
            } else {
                ok = SaveToOccupancyTreeMsg<Dtype>(m_quadtree_, 1.0, true, tree_msg);
            }
            if (ok) {
                tree_msg.header.frame_id = frame_id;
                tree_msg.header.stamp = ros::Time::now();
                m_pub_tree_.publish(tree_msg);
            }
        }

        // publish frontiers
        {
            erl_geometry_msgs::FrontierArray frontier_msg;
            if (m_is_3d_) {
                frontier_msg = OctreeFrontiersToMsg(m_frontiers_3d_, 0.1, frame_id);
            } else {
                frontier_msg = QuadtreeFrontiersToMsg(m_frontiers_2d_, 0.04, frame_id);
            }
            m_pub_frontier_.publish(frontier_msg);
        }
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "test_frontier_display_node");
    ros::NodeHandle nh("~");
    TestFrontierDisplayNode node(nh);
    ros::spin();
    return 0;
}
