#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry_msgs/ros2/occupancy_tree_msg.hpp"

#include <erl_geometry_msgs/msg/frontier_array.hpp>
#include <erl_geometry_msgs/msg/occupancy_tree_msg.hpp>
#include <rclcpp/rclcpp.hpp>

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
static erl_geometry_msgs::msg::FrontierArray
QuadtreeFrontiersToMsg(
    const std::vector<OccupancyQuadtree::Frontier> &frontiers,
    Dtype resolution,
    const std::string &frame_id,
    rclcpp::Clock &clock) {
    erl_geometry_msgs::msg::FrontierArray msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = clock.now();
    msg.dim = 2;
    msg.resolution = resolution;

    for (uint32_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        erl_geometry_msgs::msg::Frontier f_msg;
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
static erl_geometry_msgs::msg::FrontierArray
OctreeFrontiersToMsg(
    const std::vector<OccupancyOctree::Frontier> &frontiers,
    Dtype resolution,
    const std::string &frame_id,
    rclcpp::Clock &clock) {
    erl_geometry_msgs::msg::FrontierArray msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = clock.now();
    msg.dim = 3;
    msg.resolution = resolution;

    for (uint32_t fi = 0; fi < frontiers.size(); ++fi) {
        const auto &frontier = frontiers[fi];
        erl_geometry_msgs::msg::Frontier f_msg;
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

class TestFrontierDisplayNode : public rclcpp::Node {

    rclcpp::Publisher<erl_geometry_msgs::msg::FrontierArray>::SharedPtr m_pub_frontier_;
    rclcpp::Publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>::SharedPtr m_pub_tree_;
    rclcpp::TimerBase::SharedPtr m_timer_;

    bool m_is_3d_ = true;
    bool m_published_ = false;

    // 2D data
    std::shared_ptr<OccupancyQuadtree> m_quadtree_;
    std::vector<OccupancyQuadtree::Frontier> m_frontiers_2d_;

    // 3D data
    std::shared_ptr<OccupancyOctree> m_octree_;
    std::vector<OccupancyOctree::Frontier> m_frontiers_3d_;

public:
    TestFrontierDisplayNode()
        : rclcpp::Node("test_frontier_display_node") {

        this->declare_parameter("is_3d", true);
        m_is_3d_ = this->get_parameter("is_3d").as_bool();

        m_pub_frontier_ = this->create_publisher<erl_geometry_msgs::msg::FrontierArray>(
            "frontiers",
            rclcpp::QoS(1).transient_local());
        m_pub_tree_ = this->create_publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>(
            "tree",
            rclcpp::QoS(1).transient_local());

        if (m_is_3d_) {
            RCLCPP_INFO(this->get_logger(), "Building sphere octree (3D)...");
            m_octree_ = BuildSphereTree(0.1);
            m_frontiers_3d_ = m_octree_->ExtractFrontiers();
            RCLCPP_INFO(this->get_logger(), "Extracted %zu 3D frontiers", m_frontiers_3d_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "Building circle quadtree (2D)...");
            m_quadtree_ = BuildCircleTree(0.04);
            m_frontiers_2d_ = m_quadtree_->ExtractFrontiers();
            RCLCPP_INFO(this->get_logger(), "Extracted %zu 2D frontiers", m_frontiers_2d_.size());
        }

        m_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestFrontierDisplayNode::Publish, this));
    }

private:
    void
    Publish() {
        if (!m_published_) {
            RCLCPP_INFO(this->get_logger(), "Publishing tree and frontiers...");
            m_published_ = true;
        }

        const std::string frame_id = "map";

        // publish occupancy tree
        {
            erl_geometry_msgs::msg::OccupancyTreeMsg tree_msg;
            bool ok = false;
            if (m_is_3d_) {
                ok = erl::geometry::SaveToOccupancyTreeMsg<Dtype>(m_octree_, 1.0, true, tree_msg);
            } else {
                ok = erl::geometry::SaveToOccupancyTreeMsg<Dtype>(m_quadtree_, 1.0, true, tree_msg);
            }
            if (ok) {
                tree_msg.header.frame_id = frame_id;
                tree_msg.header.stamp = this->now();
                m_pub_tree_->publish(tree_msg);
            }
        }

        // publish frontiers
        {
            erl_geometry_msgs::msg::FrontierArray frontier_msg;
            if (m_is_3d_) {
                frontier_msg =
                    OctreeFrontiersToMsg(m_frontiers_3d_, 0.1, frame_id, *this->get_clock());
            } else {
                frontier_msg =
                    QuadtreeFrontiersToMsg(m_frontiers_2d_, 0.04, frame_id, *this->get_clock());
            }
            m_pub_frontier_->publish(frontier_msg);
        }
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestFrontierDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
