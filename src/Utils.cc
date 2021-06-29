/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifdef __linux__
#include <X11/Xlib.h>
#include <X11/Xresource.h>
#endif

#include "ignition/rendering/Utils.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/////////////////////////////////////////////////
float screenScalingFactor()
{
  float x{1.0f};
  float y{1.0f};
  screenScalingFactor(x, y);
  return y;
}

/////////////////////////////////////////////////
void screenScalingFactor(float &_xScale, float &_yScale)
{
  _xScale = 1.0f;
  _yScale = 1.0f;

  // todo(anyone) set device pixel ratio for high dpi displays on other
  // platforms
#ifdef __linux__
  auto closeDisplay = [](Display * display)
  {
    if (display)
      XCloseDisplay(display);
  };
  auto display =
    std::unique_ptr<Display, decltype(closeDisplay)>(
      XOpenDisplay(nullptr), closeDisplay);
  char *resourceString = XResourceManagerString(display.get());

  if (resourceString)
  {
    char *type = nullptr;
    float dpiDesktop = 0.0;

    // Need to initialize the DB before calling Xrm* functions
    XrmInitialize();

    XrmValue value;
    XrmDatabase db = XrmGetStringDatabase(resourceString);

    // Debug:
    printf("Entire DB:\n%s\n", resourceString);

    if (XrmGetResource(db, "Xft.dpi", "String", &type, &value) == True)
    {
      if (value.addr)
        dpiDesktop = atof(value.addr);
    }

    // To get the ratio we need the DPI as reported by the Xrmdatabase,
    // which takes into account desktop scaling, and the DPI computed by the
    // actual display resolution.
    //
    // dpiRes = N pixels / (M millimeters / (25.4 millimeters / 1 inch))
    //        = N pixels / (M inch / 25.4)
    //        = (N * 25.4 pixels) / M inch
    //
    // The zero values in DisplayHeight and DisplayHeightMM is the screen
    // number. A value of zero uses the default screen.
    auto xPixels = DisplayWidth(display.get(), 0);
    auto yPixels = DisplayHeight(display.get(), 0);

    auto xMM = DisplayWidthMM(display.get(), 0);
    auto yMM = DisplayHeightMM(display.get(), 0);

    auto xIn = xMM / 25.4;
    auto yIn = yMM / 25.4;

    float xDpiRes = xPixels / xIn;
    float yDpiRes = yPixels / yIn;

    if (!math::equal(dpiDesktop, 0.0f) && !math::equal(xDpiRes, 0.0f))
      _xScale = xDpiRes / dpiDesktop;

    if (!math::equal(dpiDesktop, 0.0f) && !math::equal(yDpiRes, 0.0f))
      _yScale = yDpiRes / dpiDesktop;

    if (_xScale < 1) _xScale = 1;
    if (_yScale < 1) _yScale = 1;

    // Debug:
    std::cout <<
      "DPI Desktop: " << dpiDesktop << std::endl <<
      "xPixels: " << xPixels << std::endl <<
      "xMM: " << xMM << std::endl <<
      "xIn: " << xIn << std::endl <<
      "xDpiRes: " << xDpiRes << std::endl <<
      "xScale: " << _xScale << std::endl <<
      "yPixels: " << yPixels << std::endl <<
      "yMM: " << yMM << std::endl <<
      "yIn: " << yIn << std::endl <<
      "yDpiRes: " << yDpiRes << std::endl <<
      "yScale: " << _yScale << std::endl <<
      std::endl;
  }
#endif
}

/////////////////////////////////////////////////
ignition::math::AxisAlignedBox transformAxisAlignedBox(
    const ignition::math::AxisAlignedBox &_bbox,
    const ignition::math::Pose3d &_pose)
{
  auto center = _bbox.Center();

  // Get the 8 corners of the bounding box.
  std::vector<ignition::math::Vector3d> vertices;
  vertices.push_back(center + ignition::math::Vector3d(-_bbox.XLength()/2.0,
                                                       _bbox.YLength()/2.0,
                                                       _bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(_bbox.XLength()/2.0,
                                                       _bbox.YLength()/2.0,
                                                       _bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(-_bbox.XLength()/2.0,
                                                       -_bbox.YLength()/2.0,
                                                       _bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(_bbox.XLength()/2.0,
                                                       -_bbox.YLength()/2.0,
                                                       _bbox.ZLength()/2.0));

  vertices.push_back(center + ignition::math::Vector3d(-_bbox.XLength()/2.0,
                                                       _bbox.YLength()/2.0,
                                                       -_bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(_bbox.XLength()/2.0,
                                                       _bbox.YLength()/2.0,
                                                       -_bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(-_bbox.XLength()/2.0,
                                                       -_bbox.YLength()/2.0,
                                                       -_bbox.ZLength()/2.0));
  vertices.push_back(center + ignition::math::Vector3d(_bbox.XLength()/2.0,
                                                       -_bbox.YLength()/2.0,
                                                       -_bbox.ZLength()/2.0));


  // Transform corners.
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    auto &v = vertices[i];
    v = _pose.Rot() * v + _pose.Pos();
  }

  ignition::math::Vector3d min = vertices[0];
  ignition::math::Vector3d max = vertices[0];

  // find min / max of vertices
  for (unsigned int i = 1; i < vertices.size(); ++i)
  {
    auto &v = vertices[i];

    if (min.X() > v.X())
      min.X() = v.X();
    if (max.X() < v.X())
      max.X() = v.X();
    if (min.Y() > v.Y())
      min.Y() = v.Y();
    if (max.Y() < v.Y())
      max.Y() = v.Y();
    if (min.Z() > v.Z())
      min.Z() = v.Z();
    if (max.Z() < v.Z())
      max.Z() = v.Z();
  }
  return ignition::math::AxisAlignedBox(min, max);
}
}
}
}
