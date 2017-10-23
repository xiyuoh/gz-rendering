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
#include <ignition/rendering.hh>

void BuildScene(ignition::rendering::ScenePtr _scene);

void PresentImage(ignition::rendering::ImagePtr _image);


// Global constants due to laziness
const double width = 16;
const double height = 1;
const int bytes_per_pixel = 3;


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

  std::cout << "Without hacky shader\n";
  camera->Capture(*image);
  PresentImage(image);

  std::cout << "With hacky shader\n";
  camera->Scene()->PreRender();

  // TODO Call some hacky stuff to set a custom shader on all materials
  camera->HACKSetMaterialScheme();

  camera->Render();
  camera->PostRender();
  camera->Copy(*image);
  PresentImage(image);

  return 0;
}

//////////////////////////////////////////////////
void PresentImage(ignition::rendering::ImagePtr _image)
{
  // Present the data
  unsigned char *data = _image->Data<unsigned char>();
  for (int i = 0; i < bytes_per_pixel * width * height; i += bytes_per_pixel)
  {
    unsigned long comp2 = data[i + 2] << 16;
    unsigned long comp1 = data[i + 1] << 8;
    unsigned long comp0 = data[i];
    const double distance = comp2 + comp1 + comp0;
    std::cout << distance << ", ";
  }
  std::cout << "\n";
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

  // create point light
  ignition::rendering::PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  // create green material
  ignition::rendering::MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);
  green->HACKSetShader();

  // create center visual
  ignition::rendering::VisualPtr center = _scene->CreateVisual();
  center->AddGeometry(_scene->CreateSphere());
  center->SetLocalPosition(3, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  center->SetMaterial(green);
  root->AddChild(center);

  // create red material
  ignition::rendering::MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(0.5, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(0.5, 0.5, 0.5);
  red->SetShininess(50);
  red->SetReflectivity(0);
  red->HACKSetShader();

  // create sphere visual
  ignition::rendering::VisualPtr sphere = _scene->CreateVisual();
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 2.5, 1);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  // create blue material
  ignition::rendering::MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);
  blue->HACKSetShader();

  // create box visual
  ignition::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(M_PI / 4, 0, M_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(blue);
  root->AddChild(box);

  // create white material
  ignition::rendering::MaterialPtr white = _scene->CreateMaterial();
  white->SetAmbient(0.5, 0.5, 0.5);
  white->SetDiffuse(0.8, 0.8, 0.8);
  white->SetReceiveShadows(true);
  white->SetReflectivity(0);
  white->HACKSetShader();

  // create sphere visual
  ignition::rendering::VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);
}
