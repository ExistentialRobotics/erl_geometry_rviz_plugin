// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is based on the file map_display.hpp from the rviz_default_plugins package.

#pragma once

#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

    #include <OgreMaterial.h>
    #include <OgreSharedPtr.h>
    #include <OgreTexture.h>
    #include <OgreVector.h>

#endif  // Q_MOC_RUN

#include "swatch.hpp"

#include "erl_geometry_msgs/msg/grid_map_msg.hpp"
#include "erl_geometry_msgs/msg/grid_map_update_msg.hpp"

#include <nav_msgs/msg/map_meta_data.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_default_plugins/displays/map/swatch.hpp>
#include <rviz_default_plugins/visibility_control.hpp>

namespace Ogre {
    class ManualObject;
}

namespace rviz_common {
    namespace properties {

        class EnumProperty;
        class FloatProperty;
        class IntProperty;
        class Property;
        class QuaternionProperty;
        class VectorProperty;

    }  // namespace properties
}  // namespace rviz_common

namespace erl::geometry::rviz_plugin {

    /**
     * \class GridMapDisplay
     * \brief Displays a map along the XY plane.
     */
    class RVIZ_DEFAULT_PLUGINS_PUBLIC GridMapDisplay
        : public rviz_common::MessageFilterDisplay<erl_geometry_msgs::msg::GridMapMsg> {
        Q_OBJECT

    public:
        explicit GridMapDisplay(rviz_common::DisplayContext *context);
        GridMapDisplay();
        ~GridMapDisplay() override;

        void
        onInitialize() override;
        void
        fixedFrameChanged() override;
        void
        reset() override;

        float
        getResolution() {
            return m_resolution_;
        }

        size_t
        getWidth() {
            return m_width_;
        }

        size_t
        getHeight() {
            return m_height_;
        }

        /** @brief Copy msg into m_current_map_ and call showMap(). */
        void
        processMessage(erl_geometry_msgs::msg::GridMapMsg::ConstSharedPtr msg) override;

    public Q_SLOTS:
        void
        showMap();

    Q_SIGNALS:
        /** @brief Emitted when a new map is received*/
        void
        mapUpdated();

    protected Q_SLOTS:
        void
        updateAlpha();
        void
        updateDrawUnder() const;
        void
        updatePalette();
        void
        updateBinaryThreshold();
        /** @brief Show m_current_map_ in the scene. */
        void
        transformMap();
        void
        updateMapUpdateTopic();
        void
        updateMapValueRange();

    protected:
        void
        updateTopic() override;

        void
        update(float wall_dt, float ros_dt) override;

        void
        subscribe() override;
        void
        unsubscribe() override;

        void
        onEnable() override;

        /** @brief Copy update's data into m_current_map_ and call showMap(). */
        void
        incomingUpdate(erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update);

        bool
        updateDataOutOfBounds(
            erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update) const;

        void
        updateMapDataInMemory(erl_geometry_msgs::msg::GridMapUpdateMsg::ConstSharedPtr update);

        void
        clear();

        void
        subscribeToUpdateTopic();
        void
        unsubscribeToUpdateTopic();

        void
        showValidMap();
        void
        resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
        void
        createSwatches();
        void
        doubleSwatchNumber(size_t &swatch_width, size_t &swatch_height, int &number_swatches) const;
        void
        tryCreateSwatches(
            size_t width,
            size_t height,
            float resolution,
            size_t swatch_width,
            size_t swatch_height,
            int number_swatches);
        size_t
        getEffectiveDimension(size_t map_dimension, size_t swatch_dimension, size_t position);
        void
        updateSwatches() const;

        std::vector<std::shared_ptr<erl::geometry::rviz_plugin::Swatch>> m_swatches_;
        std::vector<Ogre::TexturePtr> m_palette_textures_, m_palette_textures_binary_;
        std::vector<bool> m_color_scheme_transparency_;
        bool m_loaded_;

        float m_resolution_;
        size_t m_width_;
        size_t m_height_;
        std::string m_frame_;
        erl_geometry_msgs::msg::GridMapMsg m_current_map_;
        erl_geometry_msgs::msg::GridMapMsg m_normalized_map_;

        rclcpp::Subscription<erl_geometry_msgs::msg::GridMapUpdateMsg>::SharedPtr
            m_update_subscription_;
        rclcpp::QoS m_update_profile_;
        rclcpp::Time m_subscription_start_time_;

        rviz_common::properties::RosTopicProperty *m_update_topic_property_;
        rviz_common::properties::QosProfileProperty *m_update_profile_property_;
        rviz_common::properties::FloatProperty *m_resolution_property_;
        rviz_common::properties::IntProperty *m_width_property_;
        rviz_common::properties::IntProperty *m_height_property_;
        rviz_common::properties::StringProperty *m_encoding_property_;
        rviz_common::properties::VectorProperty *m_position_property_;
        rviz_common::properties::QuaternionProperty *m_orientation_property_;
        rviz_common::properties::FloatProperty *m_alpha_property_;
        rviz_common::properties::Property *m_draw_under_property_;
        rviz_common::properties::EnumProperty *m_color_scheme_property_;
        rviz_common::properties::BoolProperty *m_transform_timestamp_property_;
        rviz_common::properties::BoolProperty *m_binary_view_property_;
        rviz_common::properties::IntProperty *m_binary_threshold_property_;
        rviz_common::properties::FloatProperty *m_min_map_value_property_;
        rviz_common::properties::FloatProperty *m_max_map_value_property_;
        rviz_common::properties::BoolProperty *m_auto_min_max_property_;

        uint32_t m_update_messages_received_;
    };

}  // namespace erl::geometry::rviz_plugin
