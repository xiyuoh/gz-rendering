/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_RENDERING_OGRE2_OGRE2DISTORTIONPASS_HH_
#define GZ_RENDERING_OGRE2_OGRE2DISTORTIONPASS_HH_

#include <memory>
#include <vector>

#include "gz/rendering/base/BaseDistortionPass.hh"
#include "gz/rendering/ogre2/Ogre2Includes.hh"
#include "gz/rendering/ogre2/Ogre2RenderPass.hh"
#include "gz/rendering/ogre2/Export.hh"

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2DistortionPassPrivate;

    /* \class Ogre2DistortionPass Ogre2DistortionPass.hh \
     * gz/rendering/ogre2/Ogre2DistortionPass.hh
     */
    /// \brief Ogre2 implementation of the DistortionPass class
    class GZ_RENDERING_OGRE2_VISIBLE Ogre2DistortionPass :
      public BaseDistortionPass<Ogre2RenderPass>
    {
      /// \brief Constructor
      public: Ogre2DistortionPass();

      /// \brief Destructor
      public: virtual ~Ogre2DistortionPass();

      // Documentation inherited
      public: void PreRender() override;

      // Documentation inherited
      public: void CreateRenderPass() override;

      /// \brief Apply distortion model using camera coordinates projection
      /// \param[in] _in Input uv coordinate.
      /// \param[in] _center Normalized distortion center.
      /// \param[in] _k1 Radial distortion coefficient k1.
      /// \param[in] _k2 Radial distortion coefficient k2.
      /// \param[in] _k3 Radial distortion coefficient k3.
      /// \param[in] _p1 Tangential distortion coefficient p1.
      /// \param[in] _p2 Tangential distortion coefficient p2.
      /// \param[in] _width Width of the image texture in pixels.
      /// \param[in] _f Focal length in pixels.
      /// \return Distorted coordinate.
      public: static gz::math::Vector2d Distort(
                  const gz::math::Vector2d &_in,
                  const gz::math::Vector2d &_center,
                  double _k1, double _k2, double _k3,
                  double _p1, double _p2,
                  unsigned int _width, double _f);

      /// \brief get the distortion map value.
      /// \param[in] _x X component of map.
      /// \param[in] _y Y component of map.
      /// \return the distortion map value at the specified index.
      protected: gz::math::Vector2d
        DistortionMapValueClamped(int _x, int _y) const;

      /// \brief calculate the correct scale factor to "zoom" the render,
      /// cutting off black borders caused by distortion (only if the crop
      /// flag has been set).
      protected: void CalculateAndApplyDistortionScale();

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2DistortionPassPrivate> dataPtr;
    };
    }
  }
}
#endif
