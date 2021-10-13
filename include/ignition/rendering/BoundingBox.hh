/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BOUNDINGBOX_HH_
#define IGNITION_RENDERING_BOUNDINGBOX_HH_

#include <memory>
#include <vector>

#include <ignition/common/SuppressWarning.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
  class BoundingBoxPrivate;

    // TODO(anyone) Move back to BoundingBoxCamera.hh. The box shouldn't care
    // about its type, just the camera
    /// \brief BoundingBox types for Visible / Full 2D Boxes / 3D Boxes
    enum class BoundingBoxType
    {
      /// 2D box that shows the full box of occluded objects
      BBT_FULLBOX2D = 0,

      /// 2D box that shows the visible part of the
      /// occluded object
      BBT_VISIBLEBOX2D = 1,

      /// 3D oriented box
      BBT_BOX3D = 2
    };


  /// \brief 2D or 3D Bounding box. It stores the
  /// position / orientation / size info of the box and its label
  class IGNITION_RENDERING_VISIBLE BoundingBox
  {
    /// \brief Constructor
    public: BoundingBox();

    /// \brief Copy constructor
    /// \param[in] _box BoundingBox to copy.
    public: BoundingBox(const BoundingBox &_box);

    /// \brief Move constructor
    /// \param[in] _box BoundingBox to move.
    public: BoundingBox(BoundingBox &&_box) noexcept;

    /// \brief Destructor
    public: virtual ~BoundingBox();

    /// \brief Move assignment operator.
    /// \param[in] _box Heightmap box to move.
    /// \return Reference to this.
    public: BoundingBox &operator=(BoundingBox &&_box);

    /// \brief Copy Assignment operator.
    /// \param[in] _box The heightmap box to set values from.
    /// \return *this
    public: BoundingBox &operator=(const BoundingBox &_box);

    /// \brief Get the vertices of the 3D bounding box. If the bounding box
    /// type isn't 3D, an empty vector is returned
    /// \return The vertices of the 3D bounding box
    public: std::vector<math::Vector3d> Vertices() const;

    // TODO(anyone): Move members to private class and add accessor functions

    /// \brief Box type
    // TODO(anyone): Type belongs to the camera, not the box. Can this be
    // removed?
    ignition::rendering::BoundingBoxType type;

    /// \brief Center of the box in pixel coord in 2D, and camera coord in 3D
    public: ignition::math::Vector3d center;

    /// \brief Size of the box (width, height, depth), depth = 0 in 2D boxes
    public: ignition::math::Vector3d size;

    /// \brief Orientation of the 3D box in camera coord.
    /// The 2D boxes are axis aligned (orientation = 0)
    public: ignition::math::Quaterniond orientation;

    /// \brief Label of the annotated object inside the box
    public: uint32_t label;

    /// \internal
    /// \brief Private data
    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    private: std::unique_ptr<BoundingBoxPrivate> dataPtr;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
  };
}
}
}
#endif
