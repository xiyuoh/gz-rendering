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

#version ogre_glsl_ver_330

// The input texture, which is set up by the Ogre Compositor infrastructure.
vulkan_layout( ogre_t0 ) uniform texture2D RT;
vulkan_layout( ogre_t1 ) uniform texture2D distortionMap;

vulkan( layout( ogre_s0 ) uniform sampler texSampler );

vulkan( layout( ogre_P0 ) uniform Params { )
	// Scale the input texture if necessary to crop black border
	uniform vec3 scale;
vulkan( }; )

// input params from vertex shader
vulkan_layout( location = 0 )
in block
{
  vec2 uv0;
} inPs;

// final output color
vulkan_layout( location = 0 )
out vec4 fragColor;

void main()
{
  vec2 scaleCenter = vec2(0.5, 0.5); 
  vec2 inputUV = (inPs.uv0.xy - scaleCenter) / scale.xy + scaleCenter;
  vec4 mapUV = texture(vkSampler2D(distortionMap,texSampler), inputUV);

  if (mapUV.x < 0.0 || mapUV.y < 0.0)
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    fragColor = texture(vkSampler2D(RT,texSampler), mapUV.xy);
    // fragColor = texture(vkSampler2D(RT,texSampler), inPs.uv0.xy);
}

