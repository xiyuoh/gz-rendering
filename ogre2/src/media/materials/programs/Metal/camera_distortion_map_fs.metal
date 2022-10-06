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
 
// For details and documentation see: camera_distortion_map_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
};

struct Params
{
  // Mapping of undistorted to distorted uv coordinates.
  sampler2D distortionMap;
  // Scale the input texture if necessary to crop black border
  vec3 scale;
};

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float> RT [[texture(0)]],
  sampler rtSampler [[sampler(0)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  vec2 scaleCenter = vec2(0.5, 0.5);
  vec2 inputUV = (gl_TexCoord[0].xy - scaleCenter) / scale.xy + scaleCenter;
  vec4 mapUV = texture2D(distortionMap, inputUV);

  if (mapUV.x < 0.0 || mapUV.y < 0.0)
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    fragColor = texture2D(RT, mapUV.xy);

  return fragColor;
}
