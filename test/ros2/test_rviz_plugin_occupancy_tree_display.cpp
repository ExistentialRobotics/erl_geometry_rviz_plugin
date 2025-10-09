#include "erl_common/block_timer.hpp"
#include "erl_common/eigen.hpp"
#include "erl_geometry/house_expo_map_lidar_2d.hpp"
#include "erl_geometry/lidar_2d.hpp"
#include "erl_geometry/lidar_3d.hpp"
#include "erl_geometry/occupancy_octree.hpp"
#include "erl_geometry/occupancy_quadtree.hpp"
#include "erl_geometry/utils.hpp"
#include "erl_geometry_msgs/ros2/occupancy_tree_msg.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <rclcpp/rclcpp.hpp>

#define AZIMUTH_MIN         (-M_PI)
#define AZIMUTH_MAX         M_PI
#define ELEVATION_MIN       (-M_PI / 2)
#define ELEVATION_MAX       (M_PI / 2)
#define NUM_AZIMUTH_LINES   360
#define NUM_ELEVATION_LINES 181
#define OCTREE_RESOLUTION   0.1
#define QUADTREE_RESOLUTION 0.05

template<typename Dtype>
class TestOccupancyTreeGridDisplayNode3D : public rclcpp::Node {

    using Lidar3D = erl::geometry::Lidar3D<Dtype>;
    using Matrix3X = Eigen::Matrix3X<Dtype>;
    using MatrixX = Eigen::MatrixX<Dtype>;
    using Matrix4 = Eigen::Matrix4<Dtype>;
    using Matrix3 = Eigen::Matrix3<Dtype>;
    using Vector3 = Eigen::Vector3<Dtype>;
    using OccupancyOctree = erl::geometry::OccupancyOctree<Dtype>;

    rclcpp::Publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>::SharedPtr m_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::shared_ptr<Lidar3D> m_lidar_ = nullptr;
    std::vector<Matrix4> m_path_3d_;
    std::shared_ptr<OccupancyOctree> m_octree_ = nullptr;
    int m_traj_index_ = 0;
    bool m_publish_binary_msg_ = false;

public:
    TestOccupancyTreeGridDisplayNode3D()
        : rclcpp::Node("test_occupancy_tree_grid_display_node_3d") {

        // Declare parameters
        this->declare_parameter("publish_binary_msg", false);
        this->declare_parameter("mesh_file", "data/house_expo_room_1451.ply");
        this->declare_parameter("traj_file", "data/house_expo_room_1451.csv");

        m_publish_binary_msg_ = this->get_parameter("publish_binary_msg").as_bool();
        if (m_publish_binary_msg_) {
            RCLCPP_INFO(this->get_logger(), "Publish binary occupancy tree message");
        } else {
            RCLCPP_INFO(this->get_logger(), "Publish complete occupancy tree message");
        }

        // load data
        std::string mesh_file = this->get_parameter("mesh_file").as_string();
        std::string traj_file = this->get_parameter("traj_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Using mesh file: %s", mesh_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Using trajectory file: %s", traj_file.c_str());
        if (!std::filesystem::exists(mesh_file)) {
            RCLCPP_FATAL(this->get_logger(), "Mesh file does not exist: %s", mesh_file.c_str());
            rclcpp::shutdown();
            return;
        }
        if (!std::filesystem::exists(traj_file)) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Trajectory file does not exist: %s",
                traj_file.c_str());
            rclcpp::shutdown();
            return;
        }

        auto mesh_legacy = open3d::io::CreateMeshFromFile(mesh_file);
        auto mesh = open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_legacy);
        auto o3d_scene = std::make_shared<open3d::t::geometry::RaycastingScene>();
        o3d_scene->AddTriangles(mesh);

        auto lidar_setting = std::make_shared<typename Lidar3D::Setting>();
        lidar_setting->azimuth_min = AZIMUTH_MIN;
        lidar_setting->azimuth_max = AZIMUTH_MAX;
        lidar_setting->elevation_min = ELEVATION_MIN;
        lidar_setting->elevation_max = ELEVATION_MAX;
        lidar_setting->num_azimuth_lines = NUM_AZIMUTH_LINES;
        lidar_setting->num_elevation_lines = NUM_ELEVATION_LINES;
        m_lidar_ = std::make_shared<Lidar3D>(lidar_setting, o3d_scene);

        using namespace erl::common;
        m_path_3d_ = erl::geometry::ConvertPath2dTo3d<Dtype>(
            LoadEigenMatrixFromTextFile<Dtype>(traj_file, EigenTextFormat::kCsvFmt).transpose(),
            1.0);

        auto octree_setting = std::make_shared<typename OccupancyOctree::Setting>();
        octree_setting->resolution = OCTREE_RESOLUTION;
        octree_setting->log_odd_max = 10.0;
        octree_setting->SetProbabilityHit(0.95);   // log_odd_hit = 3
        octree_setting->SetProbabilityMiss(0.49);  // log_odd_miss = 0
        octree_setting->use_change_detection = true;
        m_octree_ = std::make_shared<OccupancyOctree>(octree_setting);

        // publish topic, ros timer
        m_pub_ = this->create_publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>(
            "tree",
            rclcpp::QoS(1).transient_local());
        m_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TestOccupancyTreeGridDisplayNode3D::CallbackTimer, this));
        RCLCPP_INFO(this->get_logger(), "TestOccupancyTreeGridDisplayNode3D initialized");
    }

private:
    void
    CallbackTimer() {
        if (m_traj_index_ >= static_cast<int>(m_path_3d_.size())) {
            RCLCPP_WARN(this->get_logger(), "Trajectory index out of range");
            return;
        }
        const Matrix4 pose = m_path_3d_[m_traj_index_++];
        const Matrix3 orientation = pose.template topLeftCorner<3, 3>();
        Vector3 sensor_origin = pose.template topRightCorner<3, 1>();
        MatrixX ranges;
        Matrix3X points;
        Eigen::MatrixX<Vector3> ray_directions = m_lidar_->GetRayDirectionsInFrame();
        {
            ERL_BLOCK_TIMER_MSG("Scan time");
            ranges = m_lidar_->Scan(orientation, sensor_origin);
            if (ranges.size() == 0) {
                RCLCPP_WARN(this->get_logger(), "No points scanned");
                return;
            }
            points.resize(3, ranges.size());
            long cnt_points = 0;
            for (long i = 0; i < ranges.rows(); ++i) {
                for (long j = 0; j < ranges.cols(); ++j) {
                    const Dtype& range = ranges(i, j);
                    if (!std::isfinite(range)) { continue; }
                    Vector3 point = sensor_origin + range * orientation * ray_directions(i, j);
                    points.col(cnt_points++) = point;
                }
            }
            points.conservativeResize(3, cnt_points);
        }
        {
            ERL_BLOCK_TIMER_MSG("Insert time");
            constexpr bool with_count = false;
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            m_octree_->ClearChangedKeys();
            m_octree_->InsertPointCloud(
                points,
                sensor_origin,
                0,
                -1,
                with_count,
                parallel,
                lazy_eval,
                discrete);
            if (lazy_eval) {
                m_octree_->UpdateInnerOccupancy();
                m_octree_->Prune();
            }
        }
        {
            ERL_BLOCK_TIMER_MSG("Publish time");
            using namespace erl::geometry;
            erl_geometry_msgs::msg::OccupancyTreeMsg msg;
            bool ok = SaveToOccupancyTreeMsg<Dtype>(m_octree_, m_publish_binary_msg_, msg);
            if (!ok) {
                RCLCPP_WARN(this->get_logger(), "Failed to save occupancy tree to message");
                return;
            }
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            m_pub_->publish(msg);
        }
    }
};

template<typename Dtype>
class TestOccupancyTreeGridDisplayNode2D : public rclcpp::Node {

    using Lidar2D = erl::geometry::Lidar2D;
    using Matrix2X = Eigen::Matrix2X<Dtype>;
    using Vector2 = Eigen::Vector2<Dtype>;
    using OccupancyQuadtree = erl::geometry::OccupancyQuadtree<Dtype>;

    rclcpp::Publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>::SharedPtr m_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::shared_ptr<erl::geometry::HouseExpoMapLidar2D> m_map_ = nullptr;
    std::shared_ptr<OccupancyQuadtree> m_tree_ = nullptr;
    long m_traj_index_ = 0;
    bool m_publish_binary_msg_ = false;

public:
    TestOccupancyTreeGridDisplayNode2D()
        : rclcpp::Node("test_occupancy_tree_grid_display_node_2d") {

        // Declare parameters
        this->declare_parameter("publish_binary_msg", false);
        this->declare_parameter("map_file", "data/house_expo_room_1451.json");
        this->declare_parameter("traj_file", "data/house_expo_room_1451.csv");

        m_publish_binary_msg_ = this->get_parameter("publish_binary_msg").as_bool();
        if (m_publish_binary_msg_) {
            RCLCPP_INFO(this->get_logger(), "Publish binary occupancy tree message");
        } else {
            RCLCPP_INFO(this->get_logger(), "Publish complete occupancy tree message");
        }

        // load data
        std::string map_file = this->get_parameter("map_file").as_string();
        std::string traj_file = this->get_parameter("traj_file").as_string();
        m_map_ = std::make_shared<erl::geometry::HouseExpoMapLidar2D>(
            map_file,
            traj_file,
            0.2 /* wall_thickness */,
            std::make_shared<Lidar2D::Setting>(),
            false /* add_noise */,
            0.01 /* noise_std */);
        Eigen::Vector2d map_min = m_map_->GetMapMin();
        Eigen::Vector2d map_max = m_map_->GetMapMax();
        m_map_->Translate(-(map_min + map_max) / 2.0);

        // create occupancy quadtree
        auto quadtree_setting = std::make_shared<typename OccupancyQuadtree::Setting>();
        quadtree_setting->resolution = QUADTREE_RESOLUTION;
        quadtree_setting->log_odd_max = 10.0;
        quadtree_setting->SetProbabilityHit(0.95);   // log_odd_hit = 3
        quadtree_setting->SetProbabilityMiss(0.49);  // log_odd_miss = 0
        quadtree_setting->use_change_detection = true;
        m_tree_ = std::make_shared<OccupancyQuadtree>(quadtree_setting);

        // publish topic, ros timer
        m_pub_ = this->create_publisher<erl_geometry_msgs::msg::OccupancyTreeMsg>(
            "tree",
            rclcpp::QoS(1).transient_local());
        m_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestOccupancyTreeGridDisplayNode2D::CallbackTimer, this));
        RCLCPP_INFO(this->get_logger(), "TestOccupancyTreeGridDisplayNode2D initialized");
    }

private:
    void
    CallbackTimer() {
        if (m_traj_index_ >= m_map_->Size()) {
            RCLCPP_WARN(this->get_logger(), "Trajectory index out of range");
            return;
        }
        const auto& [rotation, translation, angles, ranges] = (*m_map_)[m_traj_index_++];
        Matrix2X points(2, ranges.size());
        {
            ERL_BLOCK_TIMER_MSG("Scan time");
            long cnt = 0;
            for (long k = 0; k < ranges.size(); ++k) {
                const Dtype& angle = angles[k];
                const Dtype& range = ranges[k];
                if (!std::isfinite(range)) { continue; }
                points.col(cnt) = translation.cast<Dtype>() +
                                  rotation.cast<Dtype>() *
                                      Vector2(std::cos(angle) * range, std::sin(angle) * range);
                ++cnt;
            }
            points.conservativeResize(2, cnt);
        }
        {
            ERL_BLOCK_TIMER_MSG("Insert time");
            constexpr bool with_count = false;
            constexpr bool parallel = true;
            constexpr bool lazy_eval = true;
            constexpr bool discrete = true;
            m_tree_->InsertPointCloud(
                points,
                translation.cast<Dtype>(),
                0.0,
                -1,
                with_count,
                parallel,
                lazy_eval,
                discrete);
            if (lazy_eval) {
                m_tree_->UpdateInnerOccupancy();
                m_tree_->Prune();
            }
        }
        {
            ERL_BLOCK_TIMER_MSG("Publish time");
            using namespace erl::geometry;
            erl_geometry_msgs::msg::OccupancyTreeMsg msg;
            bool ok = SaveToOccupancyTreeMsg<Dtype>(m_tree_, m_publish_binary_msg_, msg);
            if (!ok) {
                RCLCPP_WARN(this->get_logger(), "Failed to save occupancy tree to message");
                return;
            }
            msg.header.frame_id = "map";
            msg.header.stamp = this->now();
            m_pub_->publish(msg);
        }
    }
};

class EntryNode : public rclcpp::Node {
    std::shared_ptr<void> m_node_ = nullptr;

public:
    EntryNode()
        : rclcpp::Node("test_occupancy_tree_grid_display_node") {
        // Declare parameters
        this->declare_parameter("is_double", false);
        this->declare_parameter("is_3d", true);

        bool is_double = this->get_parameter("is_double").as_bool();
        bool is_3d = this->get_parameter("is_3d").as_bool();

        if (is_double) {
            if (is_3d) {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode3D<double>>();
            } else {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode2D<double>>();
            }
        } else {
            if (is_3d) {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode3D<float>>();
            } else {
                m_node_ = std::make_shared<TestOccupancyTreeGridDisplayNode2D<float>>();
            }
        }
    }

    std::shared_ptr<rclcpp::Node>
    GetActiveNode() {
        if (auto node_3d_double =
                std::dynamic_pointer_cast<TestOccupancyTreeGridDisplayNode3D<double>>(
                    std::static_pointer_cast<TestOccupancyTreeGridDisplayNode3D<double>>(
                        m_node_))) {
            return node_3d_double;
        }
        if (auto node_3d_float =
                std::dynamic_pointer_cast<TestOccupancyTreeGridDisplayNode3D<float>>(
                    std::static_pointer_cast<TestOccupancyTreeGridDisplayNode3D<float>>(m_node_))) {
            return node_3d_float;
        }
        if (auto node_2d_double =
                std::dynamic_pointer_cast<TestOccupancyTreeGridDisplayNode2D<double>>(
                    std::static_pointer_cast<TestOccupancyTreeGridDisplayNode2D<double>>(
                        m_node_))) {
            return node_2d_double;
        }
        if (auto node_2d_float =
                std::dynamic_pointer_cast<TestOccupancyTreeGridDisplayNode2D<float>>(
                    std::static_pointer_cast<TestOccupancyTreeGridDisplayNode2D<float>>(m_node_))) {
            return node_2d_float;
        }
        return nullptr;
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto entry_node = std::make_shared<EntryNode>();
    auto active_node = entry_node->GetActiveNode();

    if (active_node) {
        rclcpp::spin(active_node);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create active node");
    }

    rclcpp::shutdown();
    return 0;
}
