/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/common/Image.hh>
#include <ignition/rendering.hh>

#include "build_config.hh"

void BuildScene(ignition::rendering::ScenePtr _scene);

void PresentImage(ignition::rendering::ImagePtr _image);


// Global constants due to laziness
const double width = 512;
const double height = 512;
const int bytes_per_pixel = 3;

const std::string vertex_shader_path = CMAKE_SOURCE_DIR "/vertex_shader.glsl";
const std::string fragment_shader_path = CMAKE_SOURCE_DIR "/fragment_shader.glsl";


//////////////////////////////////////////////////
int main()
{
  // Initialize ignition::rendering
  auto *engine = ignition::rendering::engine("ogre");
  if (!engine)
  {
    std::cerr << "Failed to load ogre\n";
    return 1;
  }
  // Create a scene and add stuff to it
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  BuildScene(scene);

  /// Create a camera
  ignition::rendering::CameraPtr camera;
  camera = scene->CreateCamera("example_custom_shaders");
  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetHFOV(1.05);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(width / height);
  camera->SetImageFormat(ignition::rendering::PF_R8G8B8);

  // Add the camera to the scene
  ignition::rendering::VisualPtr root = scene->RootVisual();
  root->AddChild(camera);

  // Render to an in-memory image
  ignition::rendering::ImagePtr image;
  image = std::make_shared<ignition::rendering::Image>(camera->CreateImage());

  // std::cout << "Without hacky shader\n";
  // camera->Capture(*image);
  // PresentImage(image);

  std::cout << "With hacky shader\n";

  //Call some stuff to set a custom shader on all materials
  // ignition::rendering::MaterialPtr depthMat = scene->CreateMaterial();
  // depthMat->SetVertexShader(vertex_shader_path);
  // depthMat->SetFragmentShader(fragment_shader_path);
  // camera->SetGlobalMaterial(depthMat);

   camera->Capture(*image);
   PresentImage(image);

  return 0;
}

//////////////////////////////////////////////////
void PresentImage(ignition::rendering::ImagePtr _image)
{
  // Present the data
  unsigned char *data = _image->Data<unsigned char>();
  // for (int i = 0; i < bytes_per_pixel * width * height; i += bytes_per_pixel)
  // {
  //   unsigned long comp2 = data[i + 2] * 100.0 * 100.0;
  //   unsigned long comp1 = data[i + 1] * 100.0;
  //   unsigned long comp0 = data[i];
  //   const double distance = comp2 + comp1 + comp0;
  //   std::cout << distance << ", ";
  // }
  // std::cout << "\n";

  ignition::common::Image image;
  image.SetFromData(data, width, height, ignition::common::Image::RGB_INT8);

  image.SavePNG("asdf.png");
}

//////////////////////////////////////////////////
void BuildScene(ignition::rendering::ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = _scene->RootVisual();

  // create directional light
  ignition::rendering::DirectionalLightPtr light0 =
    _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create white material
  ignition::rendering::MaterialPtr grey = _scene->CreateMaterial();
  grey->SetAmbient(0.5, 0.5, 0.5);
  grey->SetDiffuse(0.8, 0.8, 0.8);
  grey->SetReceiveShadows(true);
  grey->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr plane = _scene->CreateVisual();
  auto geom = _scene->CreatePlane();
  plane->AddGeometry(geom);
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(grey);
  root->AddChild(plane);

  // create shader material
  ignition::rendering::MaterialPtr shader = _scene->CreateMaterial();
  shader->SetVertexShader(vertex_shader_path);
  shader->SetFragmentShader(fragment_shader_path);
  std::cerr << "vs " << vertex_shader_path << std::endl;
  std::cerr << "fs " << fragment_shader_path << std::endl;

 // create box visual
  ignition::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(M_PI / 4, 0, M_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(shader, false);
  root->AddChild(box);
}
