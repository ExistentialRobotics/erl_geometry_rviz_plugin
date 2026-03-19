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

    #include <message_filters/subscriber.h>
    #include <OgreMaterial.h>
    #include <OgreSharedPtr.h>
    #include <OgreTexture.h>
    #include <OgreVector3.h>
    #include <ros/ros.h>

#endif  // Q_MOC_RUN

#include "swatch.hpp"

#include "erl_geometry_msgs/GridMapMsg.h"
#include "erl_geometry_msgs/GridMapUpdateMsg.h"

#include <nav_msgs/MapMetaData.h>
#include <rviz/message_filter_display.h>

namespace Ogre {
    class ManualObject;
}

namespace rviz {
    class EnumProperty;
    class FloatProperty;
    class IntProperty;
    class Property;
    class QuaternionProperty;
    class VectorProperty;
    class BoolProperty;
    class StringProperty;
    class RosTopicProperty;
}  // namespace rviz

namespace erl::geometry::rviz_plugin {

    /**
     * \class GridMapDisplay
     * \brief Displays a map along the XY plane.
     */
    class GridMapDisplay : public rviz::MessageFilterDisplay<erl_geometry_msgs::GridMapMsg> {
        Q_OBJECT

    public:
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
        processMessage(const erl_geometry_msgs::GridMapMsg::ConstPtr &msg) override;

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
        updateDrawUnder();
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
        incomingUpdate(const erl_geometry_msgs::GridMapUpdateMsg::ConstPtr &update);

        bool
        updateDataOutOfBounds(const erl_geometry_msgs::GridMapUpdateMsg::ConstPtr &update) const;

        void
        updateMapDataInMemory(const erl_geometry_msgs::GridMapUpdateMsg::ConstPtr &update);

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
        doubleSwatchNumber(size_t &swatch_width, size_t &swatch_height, int &number_swatches);
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
        updateSwatches();

        std::vector<std::shared_ptr<erl::geometry::rviz_plugin::Swatch>> m_swatches_;
        std::vector<Ogre::TexturePtr> m_palette_textures_, m_palette_textures_binary_;
        std::vector<bool> m_color_scheme_transparency_;
        bool m_loaded_;

        float m_resolution_;
        size_t m_width_;
        size_t m_height_;
        std::string m_frame_;
        erl_geometry_msgs::GridMapMsg m_current_map_;

        std::shared_ptr<message_filters::Subscriber<erl_geometry_msgs::GridMapUpdateMsg>>
            m_update_subscriber_;
        ros::Time m_subscription_start_time_;

        rviz::RosTopicProperty *m_update_topic_property_;
        rviz::FloatProperty *m_resolution_property_;
        rviz::IntProperty *m_width_property_;
        rviz::IntProperty *m_height_property_;
        rviz::StringProperty *m_encoding_property_;
        rviz::VectorProperty *m_position_property_;
        rviz::QuaternionProperty *m_orientation_property_;
        rviz::FloatProperty *m_alpha_property_;
        rviz::Property *m_draw_under_property_;
        rviz::EnumProperty *m_color_scheme_property_;
        rviz::BoolProperty *m_transform_timestamp_property_;
        rviz::BoolProperty *m_binary_view_property_;
        rviz::IntProperty *m_binary_threshold_property_;
        rviz::FloatProperty *m_min_map_value_property_;
        rviz::FloatProperty *m_max_map_value_property_;
        rviz::BoolProperty *m_auto_min_max_property_;

        uint32_t m_update_messages_received_;
    };

}  // namespace erl::geometry::rviz_plugin
