/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "erl_geometry_rviz_plugin/ros2/grid_map_display.hpp"

#include "erl_geometry_msgs/ros2/grid_map_msg_encoding.hpp"
#include "erl_geometry_rviz_plugin/ros2/palette_builder.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <rclcpp/time.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/msg_conversions.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_default_plugins/displays/map/palette_builder.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/objects/grid.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace erl::geometry::rviz_plugin {

    GridMapDisplay::GridMapDisplay()
        : m_loaded_(false),
          m_resolution_(0.0f),
          m_width_(0),
          m_height_(0),
          m_update_profile_(rclcpp::QoS(5)),
          m_update_messages_received_(0) {
        connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

        m_update_topic_property_ = new rviz_common::properties::RosTopicProperty(
            "Update Topic",
            "",
            "",
            "Topic where updates to this map display are received. "
            "This topic is automatically determined by the map topic. "
            "If the map is received on 'map_topic', the display assumes updates are received "
            "on "
            "'map_topic_updates'."
            "This can be overridden in the UI by clicking on the topic and setting the desired "
            "topic.",
            this,
            SLOT(updateMapUpdateTopic()));

        m_update_profile_property_ = new rviz_common::properties::QosProfileProperty(
            m_update_topic_property_,
            m_update_profile_);

        m_alpha_property_ = new rviz_common::properties::FloatProperty(
            "Alpha",
            0.7f,
            "Amount of transparency to apply to the map.",
            this,
            SLOT(updateAlpha()));
        m_alpha_property_->setMin(0);
        m_alpha_property_->setMax(1);

        m_color_scheme_property_ = new rviz_common::properties::EnumProperty(
            "Color Scheme",
            "map",
            "How to color the occupancy values.",
            this,
            SLOT(updatePalette()));
        // Option values here must correspond to indices in m_palette_textures_ array in
        // onInitialize() below.
        m_color_scheme_property_->addOption("map", 0);
        m_color_scheme_property_->addOption("costmap", 1);
        m_color_scheme_property_->addOption("raw", 2);
        m_color_scheme_property_->addOption("jet", 3);
        m_color_scheme_property_->addOption("jet (reversed)", 4);
        m_color_scheme_property_->addOption("hot", 5);
        m_color_scheme_property_->addOption("hot (reversed)", 6);
        m_color_scheme_property_->addOption("cool", 7);
        m_color_scheme_property_->addOption("cool (reversed)", 8);
        m_color_scheme_property_->addOption("rainbow", 9);
        m_color_scheme_property_->addOption("rainbow (reversed)", 10);
        m_color_scheme_property_->addOption("spring", 11);
        m_color_scheme_property_->addOption("spring (reversed)", 12);
        m_color_scheme_property_->addOption("summer", 13);
        m_color_scheme_property_->addOption("summer (reversed)", 14);
        m_color_scheme_property_->addOption("autumn", 15);
        m_color_scheme_property_->addOption("autumn (reversed)", 16);
        m_color_scheme_property_->addOption("winter", 17);
        m_color_scheme_property_->addOption("winter (reversed)", 18);
        m_color_scheme_property_->addOption("viridis", 19);
        m_color_scheme_property_->addOption("viridis (reversed)", 20);

        m_draw_under_property_ = new rviz_common::properties::BoolProperty(
            "Draw Behind",
            false,
            "Rendering option, controls whether or not the map is always"
            " drawn behind everything else.",
            this,
            SLOT(updateDrawUnder()));

        m_resolution_property_ = new rviz_common::properties::FloatProperty(
            "Resolution",
            0,
            "Resolution of the map. (not editable)",
            this);
        m_resolution_property_->setReadOnly(true);

        m_width_property_ = new rviz_common::properties::IntProperty(
            "Width",
            0,
            "Width of the map, in meters. (not editable)",
            this);
        m_width_property_->setReadOnly(true);

        m_height_property_ = new rviz_common::properties::IntProperty(
            "Height",
            0,
            "Height of the map, in meters. (not editable)",
            this);
        m_height_property_->setReadOnly(true);

        m_encoding_property_ = new rviz_common::properties::StringProperty(
            "Encoding",
            "",
            "Encoding of the map data. (not editable)",
            this);
        m_encoding_property_->setReadOnly(true);

        m_position_property_ = new rviz_common::properties::VectorProperty(
            "Position",
            Ogre::Vector3::ZERO,
            "Position of the bottom left corner of the map, in meters. (not editable)",
            this);
        m_position_property_->setReadOnly(true);

        m_orientation_property_ = new rviz_common::properties::QuaternionProperty(
            "Orientation",
            Ogre::Quaternion::IDENTITY,
            "Orientation of the map. (not editable)",
            this);
        m_orientation_property_->setReadOnly(true);

        m_transform_timestamp_property_ = new rviz_common::properties::BoolProperty(
            "Use Timestamp",
            false,
            "Use map header timestamp when transforming",
            this,
            SLOT(transformMap()));

        m_binary_view_property_ = new rviz_common::properties::BoolProperty(
            "Binary representation",
            false,
            "Represent the map value as either free or occupied, considering the user-defined "
            "threshold",
            this,
            SLOT(updatePalette()));

        m_binary_threshold_property_ = new rviz_common::properties::IntProperty(
            "Binary threshold",
            100,
            "Minimum value to mark cells as obstacle in the binary representation of the map",
            this,
            SLOT(updateBinaryThreshold()));
        m_binary_threshold_property_->setMin(0);
        m_binary_threshold_property_->setMax(100);

        m_min_map_value_property_ = new rviz_common::properties::FloatProperty(
            "Min map value",
            0.0f,
            "Minimum map value, used for scaling the color palette.",
            this,
            SLOT(updateMapValueRange()));

        m_max_map_value_property_ = new rviz_common::properties::FloatProperty(
            "Max map value",
            255.0f,
            "Maximum map value, used for scaling the color palette.",
            this,
            SLOT(updateMapValueRange()));

        m_auto_min_max_property_ = new rviz_common::properties::BoolProperty(
            "Auto min/max",
            false,
            "Automatically compute the min and max map values from incoming data.",
            this,
            SLOT(updateMapValueRange()));
    }

    GridMapDisplay::~GridMapDisplay() {
        unsubscribe();
        clear();
    }

    static Ogre::TexturePtr
    makePaletteTexture(std::vector<unsigned char> palette_bytes) {
        Ogre::DataStreamPtr palette_stream;
        palette_stream.reset(new Ogre::MemoryDataStream(palette_bytes.data(), 256 * 4));

        static int palette_tex_count = 0;
        std::string tex_name = "GridMapPaletteTexture" + std::to_string(palette_tex_count++);
        return Ogre::TextureManager::getSingleton().loadRawData(
            tex_name,
            "rviz_rendering",
            palette_stream,
            256,
            1,
            Ogre::PF_BYTE_RGBA,
            Ogre::TEX_TYPE_1D,
            0);
    }

    GridMapDisplay::GridMapDisplay(rviz_common::DisplayContext *context)
        : GridMapDisplay() {
        context_ = context;
        scene_manager_ = context->getSceneManager();
        scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

        m_palette_textures_.clear();
        m_palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
        m_color_scheme_transparency_.push_back(false);
        m_palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeJetPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeJetReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeHotPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeHotReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeCoolPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeCoolReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRainbowPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRainbowReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSpringPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSpringReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSummerPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSummerReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeAutumnPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeAutumnReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeWinterPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeWinterReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeViridisPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeViridisReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
    }

    void
    GridMapDisplay::onInitialize() {
        MFDClass::onInitialize();
        rviz_ros_node_ = context_->getRosNodeAbstraction();
        m_update_topic_property_->initialize(rviz_ros_node_);

        m_update_profile_property_->initialize([this](rclcpp::QoS profile) {
            this->m_update_profile_ = profile;
            updateMapUpdateTopic();
        });
        int threshold = m_binary_threshold_property_->getInt();
        // Order of palette textures here must match option indices for m_color_scheme_property_
        m_palette_textures_.clear();
        m_palette_textures_binary_.clear();
        m_palette_textures_.reserve(21);
        m_palette_textures_binary_.reserve(21);

        m_palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
        m_palette_textures_binary_.push_back(makePaletteTexture(makeMapPalette(true, threshold)));
        m_color_scheme_transparency_.push_back(false);
        m_palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
        m_palette_textures_binary_.push_back(
            makePaletteTexture(makeCostmapPalette(true, threshold)));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
        m_palette_textures_binary_.push_back(makePaletteTexture(makeRawPalette(true, threshold)));
        m_color_scheme_transparency_.push_back(true);

        m_palette_textures_.push_back(makePaletteTexture(makeJetPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeJetReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeHotPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeHotReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeCoolPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeCoolReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRainbowPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeRainbowReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSpringPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSpringReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSummerPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeSummerReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeAutumnPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeAutumnReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeWinterPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeWinterReversedPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeViridisPalette()));
        m_color_scheme_transparency_.push_back(true);
        m_palette_textures_.push_back(makePaletteTexture(makeViridisReversedPalette()));
        m_color_scheme_transparency_.push_back(true);

        m_palette_textures_binary_.insert(  // first three are the only ones that differ
            m_palette_textures_binary_.end(),
            m_palette_textures_.begin() + 3,
            m_palette_textures_.end());
    }

    void
    GridMapDisplay::updateBinaryThreshold() {
        int threshold = m_binary_threshold_property_->getInt();
        m_palette_textures_binary_[0] = makePaletteTexture(makeMapPalette(true, threshold));
        m_palette_textures_binary_[1] = makePaletteTexture(makeCostmapPalette(true, threshold));
        m_palette_textures_binary_[2] = makePaletteTexture(makeRawPalette(true, threshold));
    }

    void
    GridMapDisplay::updateTopic() {
        m_update_topic_property_->setValue(topic_property_->getTopic() + "_updates");
        MFDClass::updateTopic();
    }

    void
    GridMapDisplay::subscribe() {
        if (!isEnabled()) { return; }

        if (topic_property_->isEmpty()) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Topic",
                QString("Error subscribing: Empty topic name"));
            return;
        }

        MFDClass::subscribe();

        subscribeToUpdateTopic();
    }

    void
    GridMapDisplay::subscribeToUpdateTopic() {
        try {
            rclcpp::SubscriptionOptions sub_opts;
            sub_opts.event_callbacks.message_lost_callback = [&](rclcpp::QOSMessageLostInfo &info) {
                std::ostringstream sstm;
                sstm << "Some messages were lost:\n>\tNumber of new lost messages: "
                     << info.total_count_change
                     << " \n>\tTotal number of messages lost: " << info.total_count;
                setStatus(
                    rviz_common::properties::StatusProperty::Warn,
                    "Update Topic",
                    QString(sstm.str().c_str()));
            };

            rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
            m_update_subscription_ =
                node->template create_subscription<erl_geometry_msgs::msg::GridMapUpdateMsg>(
                    m_update_topic_property_->getTopicStd(),
                    m_update_profile_,
                    [this](const erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr message) {
                        incomingUpdate(message);
                    },
                    sub_opts);
            m_subscription_start_time_ = node->now();
            setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", "OK");
        } catch (rclcpp::exceptions::InvalidTopicNameError &e) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Update Topic",
                QString("Error subscribing: ") + e.what());
        }
    }

    void
    GridMapDisplay::unsubscribe() {
        MFDClass::unsubscribe();
        unsubscribeToUpdateTopic();
    }

    void
    GridMapDisplay::unsubscribeToUpdateTopic() {
        m_update_subscription_.reset();
    }

    void
    GridMapDisplay::updateAlpha() {
        float alpha = m_alpha_property_->getFloat();
        Ogre::SceneBlendType scene_blending;
        bool depth_write;

        rviz_rendering::MaterialManager::enableAlphaBlending(scene_blending, depth_write, alpha);

        for (const auto &swatch: m_swatches_) {
            swatch->updateAlpha(scene_blending, depth_write, alpha);
        }
    }

    void
    GridMapDisplay::updateDrawUnder() const {
        bool draw_under = m_draw_under_property_->getValue().toBool();

        if (m_alpha_property_->getFloat() >= rviz_rendering::unit_alpha_threshold) {
            for (const auto &swatch: m_swatches_) { swatch->setDepthWriteEnabled(!draw_under); }
        }

        uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
        for (const auto &swatch: m_swatches_) { swatch->setRenderQueueGroup(group); }
    }

    void
    GridMapDisplay::clear() {
        if (isEnabled()) {
            setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
        }

        if (!m_loaded_) { return; }

        m_swatches_.clear();
        m_height_ = 0;
        m_width_ = 0;
        m_resolution_ = 0.0f;

        m_loaded_ = false;
    }

    static bool
    validateFloats(const erl_geometry_msgs::msg::GridMapMsg &msg) {
        return rviz_common::validateFloats(msg.info.resolution) &&
               rviz_common::validateFloats(msg.info.origin);
    }

    static bool
    validateEncoding(uint8_t encoding) {
        switch (static_cast<GridMapEncoding>(encoding)) {
            case GridMapEncoding::INT8:
            case GridMapEncoding::UINT8:
            case GridMapEncoding::INT16:
            case GridMapEncoding::UINT16:
            case GridMapEncoding::INT32:
            case GridMapEncoding::UINT32:
            case GridMapEncoding::FLOAT32:
            case GridMapEncoding::FLOAT64:
                return true;
            default:
                return false;
        }
    }

    void
    GridMapDisplay::processMessage(erl_geometry_msgs::msg::GridMapMsg::ConstSharedPtr msg) {
        m_current_map_ = *msg;
        m_loaded_ = true;
        // updated via signal in case ros spinner is in a different thread
        Q_EMIT mapUpdated();
    }

    void
    GridMapDisplay::incomingUpdate(
        const erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update) {
        // Only update the map if we have gotten a full one first.
        if (!m_loaded_) { return; }

        ++m_update_messages_received_;
        QString topic_str = QString::number(messages_received_) + " update messages received";
        // Append topic subscription frequency if we can lock rviz_ros_node_.
        std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_interface =
            rviz_ros_node_.lock();
        if (node_interface != nullptr) {
            const double duration =
                (node_interface->get_raw_node()->now() - m_subscription_start_time_).seconds();
            const double subscription_frequency =
                static_cast<double>(messages_received_) / duration;
            topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
        }
        setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", topic_str);

        if (!validateEncoding(update->encoding)) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Update",
                "Update has invalid encoding: " + QString::number(update->encoding));
            return;
        }

        if (update->encoding != m_current_map_.encoding) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Update",
                "Update has different encoding than the current map: " +
                    QString::number(update->encoding) +
                    " != " + QString::number(m_current_map_.encoding));
            return;
        }

        if (updateDataOutOfBounds(update)) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Update",
                "Update area outside of original map area.");
            return;
        }

        updateMapDataInMemory(update);
        setStatus(rviz_common::properties::StatusProperty::Ok, "Update", "Update OK");

        // updated via signal in case ros spinner is in a different thread
        Q_EMIT mapUpdated();
    }

    bool
    GridMapDisplay::updateDataOutOfBounds(
        const erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update) const {
        return update->x < 0 || update->y < 0 ||
               m_current_map_.info.width < update->x + update->width ||
               m_current_map_.info.height < update->y + update->height;
    }

    static std::size_t
    GetScalarSize(uint8_t encoding) {
        switch (static_cast<GridMapEncoding>(encoding)) {
            case GridMapEncoding::INT8:
            case GridMapEncoding::UINT8:
                return 1;
            case GridMapEncoding::INT16:
            case GridMapEncoding::UINT16:
                return 2;
            case GridMapEncoding::INT32:
            case GridMapEncoding::UINT32:
            case GridMapEncoding::FLOAT32:
                return 4;
            case GridMapEncoding::FLOAT64:
                return 8;
            default:
                return 1;  // should never happen due to prior validation
        }
    }

    void
    GridMapDisplay::updateMapDataInMemory(
        const erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update) {
        const std::size_t scalar_size = GetScalarSize(update->encoding);
        const std::size_t stride_update = update->width * scalar_size;
        const std::size_t stride_map = m_current_map_.info.width * scalar_size;
        for (size_t y = 0; y < update->height; y++) {
            auto offset = update->data.begin() + y * stride_update;
            std::copy(
                offset,
                offset + stride_update,
                m_current_map_.data.begin() + (update->y + y) * stride_map + update->x);
        }
    }

    void
    GridMapDisplay::createSwatches() {
        size_t width = m_current_map_.info.width;
        size_t height = m_current_map_.info.height;
        float resolution = m_current_map_.info.resolution;

        size_t swatch_width = width;
        size_t swatch_height = height;
        int number_swatches = 1;
        // One swatch can have up to 2^16 * 2^16 pixel (8 bit texture, i.e. 4GB of data)
        // Since the width and height are separately limited by 2^16 it might be necessary to
        // have several pieces, however more than 8 swatches is probably unnecessary due to
        // memory limitations
        const size_t maximum_number_swatch_splittings = 4;

        for (size_t i = 0; i < maximum_number_swatch_splittings; ++i) {
            RVIZ_COMMON_LOG_INFO_STREAM(
                "Trying to create a map of size " << width << " x " << height << " using "
                                                  << number_swatches << " swatches");
            m_swatches_.clear();
            try {
                tryCreateSwatches(
                    width,
                    height,
                    resolution,
                    swatch_width,
                    swatch_height,
                    number_swatches);
                updateDrawUnder();
                return;
            } catch (Ogre::InvalidParametersException &) {
                doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
            } catch (Ogre::RenderingAPIException &) {
                // This exception seems no longer thrown on some systems. May still be relevant
                // for others.
                doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
            }
        }
        RVIZ_COMMON_LOG_ERROR_STREAM(
            "Creating " << number_swatches
                        << "failed. "
                           "This map is too large to be displayed by RViz.");
        m_swatches_.clear();
    }

    void
    GridMapDisplay::doubleSwatchNumber(
        size_t &swatch_width,
        size_t &swatch_height,
        int &number_swatches) const {
        RVIZ_COMMON_LOG_ERROR_STREAM(
            "Failed to create map using " << number_swatches
                                          << " swatches. "
                                             "At least one swatch seems to need too much memory");
        if (swatch_width > swatch_height) {
            swatch_width /= 2;
        } else {
            swatch_height /= 2;
        }
        number_swatches *= 2;
    }

    void
    GridMapDisplay::tryCreateSwatches(
        size_t width,
        size_t height,
        float resolution,
        size_t swatch_width,
        size_t swatch_height,
        int number_swatches) {
        size_t x = 0;
        size_t y = 0;
        for (int i = 0; i < number_swatches; i++) {
            size_t effective_width = getEffectiveDimension(width, swatch_width, x);
            size_t effective_height = getEffectiveDimension(height, swatch_height, y);

            m_swatches_.push_back(
                std::make_shared<Swatch>(
                    scene_manager_,
                    scene_node_,
                    x,
                    y,
                    effective_width,
                    effective_height,
                    resolution,
                    m_min_map_value_property_->getFloat(),
                    m_max_map_value_property_->getFloat(),
                    m_draw_under_property_->getValue().toBool()));

            m_swatches_[i]->updateData(m_current_map_);

            x += effective_width;
            if (x >= width) {
                x = 0;
                y += effective_height;
            }
        }
        updateAlpha();
    }

    size_t
    GridMapDisplay::getEffectiveDimension(
        size_t map_dimension,
        size_t swatch_dimension,
        size_t position) {
        // Last swatch is bigger than swatch_dimension for odd numbers.
        // subtracting the swatch_dimension in the LHS handles this case.
        return map_dimension - position - swatch_dimension >= swatch_dimension
                   ? swatch_dimension
                   : map_dimension - position;
    }

    void
    GridMapDisplay::showMap() {
        if (m_current_map_.data.empty()) { return; }

        if (!validateFloats(m_current_map_)) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Map",
                "Message contained invalid floating point values (nans or infs)");
            return;
        }

        size_t width = m_current_map_.info.width;
        size_t height = m_current_map_.info.height;

        if (width * height == 0) {
            std::string message =
                "Map is zero-sized (" + std::to_string(width) + "x" + std::to_string(height) + ")";
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Map",
                QString::fromStdString(message));
            return;
        }

        const std::size_t scalar_size = GetScalarSize(m_current_map_.encoding);
        if (width * height * scalar_size != m_current_map_.data.size()) {
            std::string message =
                "Data size doesn't match width*height: width = " + std::to_string(width) +
                ", height = " + std::to_string(height) +
                ", scalar size = " + std::to_string(scalar_size) +
                ", data size = " + std::to_string(m_current_map_.data.size());
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Map",
                QString::fromStdString(message));
            return;
        }

        setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");

        RVIZ_COMMON_LOG_DEBUG_STREAM(
            "Received a " << m_current_map_.info.width << " X " << m_current_map_.info.height
                          << " map @ " << m_current_map_.info.resolution << "m/pix\n");

        showValidMap();
    }

    static std::string
    GetEncodingName(uint8_t encoding) {
        switch (static_cast<GridMapEncoding>(encoding)) {
            case GridMapEncoding::INT8:
                return "INT8";
            case GridMapEncoding::UINT8:
                return "UINT8";
            case GridMapEncoding::INT16:
                return "INT16";
            case GridMapEncoding::UINT16:
                return "UINT16";
            case GridMapEncoding::INT32:
                return "INT32";
            case GridMapEncoding::UINT32:
                return "UINT32";
            case GridMapEncoding::FLOAT32:
                return "FLOAT32";
            case GridMapEncoding::FLOAT64:
                return "FLOAT64";
            default:
                return "UNKNOWN";
        }
    }

    void
    GridMapDisplay::showValidMap() {
        size_t width = m_current_map_.info.width;
        size_t height = m_current_map_.info.height;

        float resolution = m_current_map_.info.resolution;

        resetSwatchesIfNecessary(width, height, resolution);

        m_frame_ = m_current_map_.header.frame_id;
        if (m_frame_.empty()) { m_frame_ = "/map"; }

        updateSwatches();

        setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "Map OK");
        updatePalette();

        m_resolution_property_->setValue(resolution);
        m_width_property_->setValue(static_cast<unsigned int>(width));
        m_height_property_->setValue(static_cast<unsigned int>(height));
        m_encoding_property_->setValue(
            QString::fromStdString(GetEncodingName(m_current_map_.encoding)));

        m_position_property_->setVector(
            rviz_common::pointMsgToOgre(m_current_map_.info.origin.position));
        m_orientation_property_->setQuaternion(
            rviz_common::quaternionMsgToOgre(m_current_map_.info.origin.orientation));

        transformMap();

        updateDrawUnder();

        updateMapValueRange();

        context_->queueRender();
    }

    void
    GridMapDisplay::resetSwatchesIfNecessary(size_t width, size_t height, float resolution) {
        if (width != m_width_ || height != m_height_ || m_resolution_ != resolution) {
            createSwatches();
            m_width_ = width;
            m_height_ = height;
            m_resolution_ = resolution;
        }
    }

    void
    GridMapDisplay::updateSwatches() const {
        for (const auto &swatch: m_swatches_) {
            swatch->updateData(m_current_map_);

            Ogre::Pass *pass = swatch->getTechniquePass();
            Ogre::TextureUnitState *tex_unit = nullptr;
            if (pass->getNumTextureUnitStates() > 0) {
                tex_unit = pass->getTextureUnitState(0);
            } else {
                tex_unit = pass->createTextureUnitState();
            }

            tex_unit->setTextureName(swatch->getTextureName());
            tex_unit->setTextureFiltering(Ogre::TFO_NONE);
            swatch->setVisible(true);
            swatch->resetOldTexture();
        }
    }

    void
    GridMapDisplay::updatePalette() {
        bool binary = m_binary_view_property_->getBool();

        int palette_index = m_color_scheme_property_->getOptionInt();

        for (const auto &swatch: m_swatches_) {
            Ogre::Pass *pass = swatch->getTechniquePass();
            Ogre::TextureUnitState *palette_tex_unit = nullptr;
            if (pass->getNumTextureUnitStates() > 1) {
                palette_tex_unit = pass->getTextureUnitState(1);
            } else {
                palette_tex_unit = pass->createTextureUnitState();
            }
            if (binary) {
                palette_tex_unit->setTexture(m_palette_textures_binary_[palette_index]);
            } else {
                palette_tex_unit->setTexture(m_palette_textures_[palette_index]);
            }
            palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
        }

        updateAlpha();
        updateDrawUnder();
    }

    void
    GridMapDisplay::transformMap() {
        if (!m_loaded_) { return; }

        rclcpp::Time transform_time = context_->getClock()->now();

        if (m_transform_timestamp_property_->getBool()) {
            transform_time = rclcpp::Time(m_current_map_.header.stamp, RCL_ROS_TIME);
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->transform(
                m_frame_,
                transform_time,
                m_current_map_.info.origin,
                position,
                orientation) &&
            !context_->getFrameManager()->transform(
                m_frame_,
                rclcpp::Time(0, 0, context_->getClock()->get_clock_type()),
                m_current_map_.info.origin,
                position,
                orientation)) {
            setMissingTransformToFixedFrame(m_frame_);
            scene_node_->setVisible(false);
        } else {
            setTransformOk();

            scene_node_->setPosition(position);
            scene_node_->setOrientation(orientation);
        }
    }

    void
    GridMapDisplay::fixedFrameChanged() {
        transformMap();
    }

    void
    GridMapDisplay::reset() {
        MFDClass::reset();
        m_update_messages_received_ = 0;
        clear();
    }

    void
    GridMapDisplay::update(float wall_dt, float ros_dt) {
        (void) wall_dt;
        (void) ros_dt;

        transformMap();
    }

    void
    GridMapDisplay::onEnable() {
        MFDClass::onEnable();
        setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
    }

    void
    GridMapDisplay::updateMapUpdateTopic() {
        unsubscribeToUpdateTopic();
        reset();
        subscribeToUpdateTopic();
        context_->queueRender();
    }

    template<typename T>
    static std::pair<float, float>
    getMinMax(const std::vector<uint8_t> &data) {
        if (data.empty()) { return {0.0f, 0.0f}; }

        float min_val = std::numeric_limits<float>::infinity();
        float max_val = -std::numeric_limits<float>::infinity();

        const T *data_ptr = reinterpret_cast<const T *>(data.data());
        size_t num_elements = data.size() / sizeof(T);

        for (size_t i = 0; i < num_elements; ++i) {
            auto value = static_cast<float>(data_ptr[i]);
            min_val = std::min(min_val, value);
            max_val = std::max(max_val, value);
        }

        return {min_val, max_val};
    }

    void
    GridMapDisplay::updateMapValueRange() {
        float min_value = m_min_map_value_property_->getFloat();
        float max_value = m_max_map_value_property_->getFloat();

        if (m_auto_min_max_property_->getBool()) {
            if (!m_loaded_) { return; }  // wait for first map
            GridMapEncoding encoding = static_cast<GridMapEncoding>(m_current_map_.encoding);
            switch (encoding) {
                case GridMapEncoding::INT8:
                    std::tie(min_value, max_value) = getMinMax<int8_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::UINT8:
                    std::tie(min_value, max_value) = getMinMax<uint8_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::INT16:
                    std::tie(min_value, max_value) = getMinMax<int16_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::UINT16:
                    std::tie(min_value, max_value) = getMinMax<uint16_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::INT32:
                    std::tie(min_value, max_value) = getMinMax<int32_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::UINT32:
                    std::tie(min_value, max_value) = getMinMax<uint32_t>(m_current_map_.data);
                    break;
                case GridMapEncoding::FLOAT32:
                    std::tie(min_value, max_value) = getMinMax<float>(m_current_map_.data);
                    break;
                case GridMapEncoding::FLOAT64:
                    std::tie(min_value, max_value) = getMinMax<double>(m_current_map_.data);
                    break;
                default:
                    setStatus(
                        rviz_common::properties::StatusProperty::Error,
                        "Map Value Range",
                        "Cannot determine min/max for unknown encoding.");
                    return;
            }
            m_min_map_value_property_->setValue(min_value);
            m_max_map_value_property_->setValue(max_value);
        }

        if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Map Value Range",
                "Map value range must be finite.");
            return;
        }

        if (min_value >= max_value) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Map Value Range",
                "Minimum map value must be smaller than maximum map value.");
            return;
        }

        setStatus(rviz_common::properties::StatusProperty::Ok, "Map Value Range", "OK");

        for (const auto &swatch: m_swatches_) { swatch->setValueRange(min_value, max_value); }
    }

}  // namespace erl::geometry::rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(erl::geometry::rviz_plugin::GridMapDisplay, rviz_common::Display)
