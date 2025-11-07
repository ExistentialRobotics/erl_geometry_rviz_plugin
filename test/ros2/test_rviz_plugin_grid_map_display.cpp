#include "erl_geometry_msgs/msg/grid_map_msg.hpp"
#include "erl_geometry_msgs/ros2/grid_map_msg_encoding.hpp"

#include <rclcpp/rclcpp.hpp>

// send a 2D sinisoidal wave via GridMapMsg

template<typename T>
class TestGridMapDisplayNode : public rclcpp::Node {
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
    std::string m_topic_reliability_ = "reliable";
    std::string m_topic_durability_ = "volatile";

    rclcpp::Publisher<erl_geometry_msgs::msg::GridMapMsg>::SharedPtr m_pub_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    double m_phase_x_ = 0.0;
    double m_phase_y_ = 0.0;

public:
    TestGridMapDisplayNode()
        : rclcpp::Node("test_grid_map_display_node") {

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
            RCLCPP_ERROR(this->get_logger(), "Unsupported template type");
            throw std::runtime_error("Unsupported template type");
        }

        this->declare_parameter("magnitude", m_magnitude_);
        this->declare_parameter("frequency", m_freq_);
        this->declare_parameter("publish_rate", m_publish_rate_);
        this->declare_parameter("resolution", m_resolution_);
        this->declare_parameter("x", m_x_);
        this->declare_parameter("y", m_y_);
        this->declare_parameter("width", m_width_);
        this->declare_parameter("height", m_height_);

        m_magnitude_ = this->get_parameter("magnitude").as_double();
        m_freq_ = this->get_parameter("frequency").as_double();
        m_publish_rate_ = this->get_parameter("publish_rate").as_double();
        m_resolution_ = this->get_parameter("resolution").as_double();
        m_x_ = this->get_parameter("x").as_double();
        m_y_ = this->get_parameter("y").as_double();
        m_width_ = this->get_parameter("width").as_int();
        m_height_ = this->get_parameter("height").as_int();

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        if (m_topic_reliability_ == "reliable") {
            qos.reliable();
        } else if (m_topic_reliability_ == "best_effort") {
            qos.best_effort();
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported reliability: %s",
                m_topic_reliability_.c_str());
        }
        if (m_topic_durability_ == "volatile") {
            qos.durability_volatile();
        } else if (m_topic_durability_ == "transient_local") {
            qos.transient_local();
        } else {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported durability: %s",
                m_topic_durability_.c_str());
        }

        m_pub_ = this->create_publisher<erl_geometry_msgs::msg::GridMapMsg>(m_topic_, qos);
        m_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / m_publish_rate_),
            std::bind(&TestGridMapDisplayNode::Callback, this));
        RCLCPP_INFO(this->get_logger(), "TestGridMapDisplayNode initialized");
    }

    void
    Callback() {
        erl_geometry_msgs::msg::GridMapMsg msg;
        msg.header.stamp = this->now();
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
        m_pub_->publish(msg);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto temp_node = rclcpp::Node::make_shared("test_grid_map_display_node");
    temp_node->declare_parameter("encoding", "float");
    std::string encoding = temp_node->get_parameter("encoding").as_string();
    temp_node.reset();  // release the temporary node

    std::shared_ptr<rclcpp::Node> node;
    if (encoding == "float") {
        node = std::make_shared<TestGridMapDisplayNode<float>>();
    } else if (encoding == "double") {
        node = std::make_shared<TestGridMapDisplayNode<double>>();
    } else if (encoding == "int8") {
        node = std::make_shared<TestGridMapDisplayNode<int8_t>>();
    } else if (encoding == "uint8") {
        node = std::make_shared<TestGridMapDisplayNode<uint8_t>>();
    } else if (encoding == "int16") {
        node = std::make_shared<TestGridMapDisplayNode<int16_t>>();
    } else if (encoding == "uint16") {
        node = std::make_shared<TestGridMapDisplayNode<uint16_t>>();
    } else if (encoding == "int32") {
        node = std::make_shared<TestGridMapDisplayNode<int32_t>>();
    } else if (encoding == "uint32") {
        node = std::make_shared<TestGridMapDisplayNode<uint32_t>>();
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("test_grid_map_display_node"),
            "Unsupported encoding: %s",
            encoding.c_str());
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
