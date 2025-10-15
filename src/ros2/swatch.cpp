/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

// This file is based on the file swatch.cpp from the rviz_default_plugins package.

#include "erl_geometry_rviz_plugin/ros2/swatch.hpp"

#include "erl_geometry_msgs/ros2/grid_map_msg_encoding.hpp"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <rviz_rendering/custom_parameter_indices.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace erl::geometry::rviz_plugin {

    // Helper class to set alpha parameter on all renderables.
    class AlphaSetter : public Ogre::Renderable::Visitor {
    public:
        explicit AlphaSetter(float alpha)
            : alpha_vec_(alpha, alpha, alpha, alpha) {}

        void
        visit(Ogre::Renderable *rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any *pAny)
            override {
            (void) lodIndex;
            (void) isDebug;
            (void) pAny;

            rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
        }

    private:
        Ogre::Vector4 alpha_vec_;
    };

    size_t Swatch::m_material_count_ = 0;
    size_t Swatch::m_map_count_ = 0;
    size_t Swatch::m_node_count_ = 0;
    size_t Swatch::m_texture_count_ = 0;

    Swatch::Swatch(
        Ogre::SceneManager *scene_manager,
        Ogre::SceneNode *parent_scene_node,
        size_t x,
        size_t y,
        size_t width,
        size_t height,
        float resolution,
        float map_value_min,
        float map_value_max,
        bool draw_under)
        : m_scene_manager_(scene_manager),
          m_parent_scene_node_(parent_scene_node),
          m_manual_object_(nullptr),
          m_x_(x),
          m_y_(y),
          m_width_(width),
          m_height_(height),
          m_map_value_min_(map_value_min),
          m_map_value_max_(map_value_max) {
        setupMaterial();
        setupSceneNodeWithManualObject();

        m_scene_node_->setPosition(x * resolution, y * resolution, 0);
        m_scene_node_->setScale(width * resolution, height * resolution, 1.0);

        if (draw_under) { m_manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4); }

        // don't show map until the plugin is actually enabled
        m_manual_object_->setVisible(false);
    }

    Swatch::~Swatch() { m_scene_manager_->destroyManualObject(m_manual_object_); }

    void
    Swatch::updateAlpha(const Ogre::SceneBlendType &sceneBlending, bool depth_write, float alpha) {
        m_material_->setSceneBlending(sceneBlending);
        m_material_->setDepthWriteEnabled(depth_write);
        if (m_manual_object_) {
            AlphaSetter alpha_setter(alpha);
            m_manual_object_->visitRenderables(&alpha_setter);
        }
    }

    void
    Swatch::setValueRange(float map_value_min, float map_value_max) {
        m_map_value_min_ = map_value_min;
        m_map_value_max_ = map_value_max;
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

    template<typename T>
    void
    NormalizeMapDataTemplate(
        const T *input_data,
        float map_value_min,
        float map_value_max,
        std::size_t length,
        uint8_t *output_buf) {

        float range = map_value_max - map_value_min;
        for (std::size_t i = 0; i < length; ++i, ++input_data, ++output_buf) {
            float value = static_cast<float>(*input_data);
            if (value <= map_value_min) {
                *output_buf = 0;
            } else if (value >= map_value_max) {
                *output_buf = 255;
            } else {
                *output_buf = static_cast<uint8_t>(((value - map_value_min) / range) * 255.0f);
            }
        }
    }

    static void
    NormalizeMapData(
        const uint8_t *input_data,
        uint8_t encoding,
        float map_value_min,
        float map_value_max,
        std::size_t length,
        uint8_t *output_buf) {

        switch (static_cast<GridMapEncoding>(encoding)) {
            case GridMapEncoding::INT8:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const int8_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::UINT8:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const uint8_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::INT16:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const int16_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::UINT16:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const uint16_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::INT32:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const int32_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::UINT32:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const uint32_t *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::FLOAT32:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const float *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            case GridMapEncoding::FLOAT64:
                NormalizeMapDataTemplate(
                    reinterpret_cast<const double *>(input_data),
                    map_value_min,
                    map_value_max,
                    length,
                    output_buf);
                break;
            default:
                // should never happen due to prior validation
                std::fill(output_buf, output_buf + length, 255);
        }
    }

    void
    Swatch::updateData(const erl_geometry_msgs::msg::GridMapMsg &map) {
        size_t pixels_size = m_width_ * m_height_;
        size_t map_size = map.info.width * map.info.height;
        size_t map_width = map.info.width;

        auto pixels = std::vector<unsigned char>(pixels_size, 255);
        GridMapEncoding encoding = static_cast<GridMapEncoding>(map.encoding);

        uint8_t *pixel_data = pixels.data();
        const std::size_t scalar_size = GetScalarSize(map.encoding);
        for (std::size_t map_row = m_y_; map_row < m_y_ + m_height_; map_row++) {
            size_t pixel_index = map_row * map_width + m_x_;
            size_t pixels_to_copy = std::min(m_width_, map_size - pixel_index);

            const uint8_t *map_data_start = map.data.data() + pixel_index * scalar_size;
            NormalizeMapData(
                map_data_start,
                map.encoding,
                m_map_value_min_,
                m_map_value_max_,
                pixels_to_copy,
                pixel_data);
            pixel_data += pixels_to_copy;
            if (pixel_index + pixels_to_copy >= map_size) { break; }
        }

        Ogre::DataStreamPtr pixel_stream(new Ogre::MemoryDataStream(pixels.data(), pixels_size));

        resetTexture(pixel_stream);
        resetOldTexture();
    }

    void
    Swatch::setVisible(bool visible) {
        if (m_manual_object_) { m_manual_object_->setVisible(visible); }
    }

    void
    Swatch::resetOldTexture() {
        if (m_old_texture_) {
            Ogre::TextureManager::getSingleton().remove(m_old_texture_);
            m_old_texture_.reset();
        }
    }

    void
    Swatch::setRenderQueueGroup(uint8_t group) {
        if (m_manual_object_) { m_manual_object_->setRenderQueueGroup(group); }
    }

    void
    Swatch::setDepthWriteEnabled(bool depth_write_enabled) {
        if (m_material_) { m_material_->setDepthWriteEnabled(depth_write_enabled); }
    }

    Ogre::Pass *
    Swatch::getTechniquePass() {
        if (m_material_) { return m_material_->getTechnique(0)->getPass(0); }
        return nullptr;
    }

    std::string
    Swatch::getTextureName() {
        if (m_texture_) { return m_texture_->getName(); }
        return "";
    }

    void
    Swatch::resetTexture(Ogre::DataStreamPtr &pixel_stream) {
        m_old_texture_ = m_texture_;

        m_texture_ = Ogre::TextureManager::getSingleton().loadRawData(
            "GridMapTexture" + std::to_string(m_texture_count_++),
            "rviz_rendering",
            pixel_stream,
            static_cast<uint16_t>(m_width_),
            static_cast<uint16_t>(m_height_),
            Ogre::PF_L8,
            Ogre::TEX_TYPE_2D,
            0);
    }

    void
    Swatch::setupMaterial() {
        m_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
        m_material_ = m_material_->clone("GridMapMaterial" + std::to_string(m_material_count_++));

        m_material_->setReceiveShadows(false);
        m_material_->getTechnique(0)->setLightingEnabled(false);
        m_material_->setDepthBias(-16.0f, 0.0f);
        m_material_->setCullingMode(Ogre::CULL_NONE);
        m_material_->setDepthWriteEnabled(false);
    }

    void
    Swatch::setupSceneNodeWithManualObject() {
        m_manual_object_ =
            m_scene_manager_->createManualObject("GridMapObject" + std::to_string(m_map_count_++));

        m_scene_node_ = m_parent_scene_node_->createChildSceneNode(
            "GridMapNodeObject" + std::to_string(m_node_count_++));
        m_scene_node_->attachObject(m_manual_object_);

        setupSquareManualObject();
    }

    void
    Swatch::setupSquareManualObject() {
        m_manual_object_->begin(
            m_material_->getName(),
            Ogre::RenderOperation::OT_TRIANGLE_LIST,
            "rviz_rendering");

        // first triangle
        addPointWithPlaneCoordinates(0.0f, 0.0f);
        addPointWithPlaneCoordinates(1.0f, 1.0f);
        addPointWithPlaneCoordinates(0.0f, 1.0f);

        // second triangle
        addPointWithPlaneCoordinates(0.0f, 0.0f);
        addPointWithPlaneCoordinates(1.0f, 0.0f);
        addPointWithPlaneCoordinates(1.0f, 1.0f);

        m_manual_object_->end();
    }

    void
    Swatch::addPointWithPlaneCoordinates(float x, float y) {
        m_manual_object_->position(x, y, 0.0f);
        m_manual_object_->textureCoord(x, y);
        m_manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

}  // namespace erl::geometry::rviz_plugin
