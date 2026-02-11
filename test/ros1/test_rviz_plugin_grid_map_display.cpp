#include "erl_geometry_msgs/GridMapMsg.h"
#include "erl_geometry_msgs/ros1/grid_map_msg_encoding.hpp"

#include <ros/ros.h>

// send a 2D sinisoidal wave via GridMapMsg

template<typename T>
class TestGridMapDisplayNode {
    double m_magnitude_ = 10.0;
    double m_freq_ = 0.2;
    double m_publish_rate_ = 1.0;
    double m_resolution_ = 0.1;
    double m_x_ = 0.0;
    double m_y_ = 0.0;
    int m_width_ = 100;
    int m_height_ = 100;
    erl::geometry::GridMapEncoding m_encoding_ = erl::geometry::GridMapEncoding::FLOAT32;

    std::string m_topic_ = "grid_map";

    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Timer m_timer_;
    double m_phase_x_ = 0.0;
    double m_phase_y_ = 0.0;

public:
    TestGridMapDisplayNode()
        : m_nh_("~") {

        if constexpr (std::is_same<T, float>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::FLOAT32;
        } else if constexpr (std::is_same<T, double>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::FLOAT64;
        } else if constexpr (std::is_same<T, int8_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::INT8;
        } else if constexpr (std::is_same<T, uint8_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::UINT8;
        } else if constexpr (std::is_same<T, int16_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::INT16;
        } else if constexpr (std::is_same<T, uint16_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::UINT16;
        } else if constexpr (std::is_same<T, int32_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::INT32;
        } else if constexpr (std::is_same<T, uint32_t>::value) {
            m_encoding_ = erl::geometry::GridMapEncoding::UINT32;
        } else {
            ROS_ERROR("Unsupported template type");
            throw std::runtime_error("Unsupported template type");
        }

        m_nh_.param("magnitude", m_magnitude_, m_magnitude_);
        m_nh_.param("frequency", m_freq_, m_freq_);
        m_nh_.param("publish_rate", m_publish_rate_, m_publish_rate_);
        m_nh_.param("resolution", m_resolution_, m_resolution_);
        m_nh_.param("x", m_x_, m_x_);
        m_nh_.param("y", m_y_, m_y_);
        m_nh_.param("width", m_width_, m_width_);
        m_nh_.param("height", m_height_, m_height_);

        m_pub_ = m_nh_.advertise<erl_geometry_msgs::GridMapMsg>(m_topic_, 10);
        m_timer_ = m_nh_.createTimer(
            ros::Duration(1.0 / m_publish_rate_),
            &TestGridMapDisplayNode::Callback, this);
        ROS_INFO("TestGridMapDisplayNode initialized");
    }

    void
    Callback(const ros::TimerEvent&) {
        erl_geometry_msgs::GridMapMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.info.resolution = m_resolution_;
        msg.info.width = m_width_;
        msg.info.height = m_height_;
        msg.info.origin.position.x = m_x_;
        msg.info.origin.position.y = m_y_;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.encoding = static_cast<uint8_t>(m_encoding_);
        size_t num_cells = static_cast<size_t>(m_width_) * static_cast<size_t>(m_height_);
        msg.data.resize(num_cells * sizeof(T));
        T *data_ptr = reinterpret_cast<T *>(msg.data.data());

        for (int j = 0; j < m_height_; ++j) {
            for (int i = 0; i < m_width_; ++i) {
                double x = m_x_ + (i + 0.5) * m_resolution_;
                double y = m_y_ + (j + 0.5) * m_resolution_;
                double value = m_magnitude_ * std::sin(m_freq_ * x + m_phase_x_) *
                               std::cos(m_freq_ * y + m_phase_y_);
                data_ptr[j * m_width_ + i] = static_cast<T>(value);
            }
        }

        m_phase_x_ += m_freq_ * m_resolution_;
        m_phase_y_ += m_freq_ * m_resolution_;
        m_pub_.publish(msg);
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "test_grid_map_display_node");

    ros::NodeHandle nh("~");
    std::string encoding;
    nh.param("encoding", encoding, std::string("float"));

    std::shared_ptr<TestGridMapDisplayNode<float>> node_float;
    std::shared_ptr<TestGridMapDisplayNode<double>> node_double;
    std::shared_ptr<TestGridMapDisplayNode<int8_t>> node_int8;
    std::shared_ptr<TestGridMapDisplayNode<uint8_t>> node_uint8;
    std::shared_ptr<TestGridMapDisplayNode<int16_t>> node_int16;
    std::shared_ptr<TestGridMapDisplayNode<uint16_t>> node_uint16;
    std::shared_ptr<TestGridMapDisplayNode<int32_t>> node_int32;
    std::shared_ptr<TestGridMapDisplayNode<uint32_t>> node_uint32;

    if (encoding == "float") {
        node_float = std::make_shared<TestGridMapDisplayNode<float>>();
    } else if (encoding == "double") {
        node_double = std::make_shared<TestGridMapDisplayNode<double>>();
    } else if (encoding == "int8") {
        node_int8 = std::make_shared<TestGridMapDisplayNode<int8_t>>();
    } else if (encoding == "uint8") {
        node_uint8 = std::make_shared<TestGridMapDisplayNode<uint8_t>>();
    } else if (encoding == "int16") {
        node_int16 = std::make_shared<TestGridMapDisplayNode<int16_t>>();
    } else if (encoding == "uint16") {
        node_uint16 = std::make_shared<TestGridMapDisplayNode<uint16_t>>();
    } else if (encoding == "int32") {
        node_int32 = std::make_shared<TestGridMapDisplayNode<int32_t>>();
    } else if (encoding == "uint32") {
        node_uint32 = std::make_shared<TestGridMapDisplayNode<uint32_t>>();
    } else {
        ROS_ERROR("Unsupported encoding: %s", encoding.c_str());
        return -1;
    }

    ros::spin();
    return 0;
}
