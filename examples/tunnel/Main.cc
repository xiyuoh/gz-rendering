/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include <iostream>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;


const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");


//////////////////////////////////////////////////
void buildScene(ScenePtr _scene)
{
  // initialize _scene
//   _scene->SetAmbientLight(0.0, 0.0, 0.0);
   _scene->SetAmbientLight(0.08, 0.08, 0.08);
//   _scene->SetAmbientLight(0.1, 0.1, 0.1);
//  _scene->SetAmbientLight(1.0, 1.0, 1.0);
  VisualPtr root = _scene->RootVisual();

  common::MeshManager *meshManager = common::MeshManager::Instance();

  // create a mesh
  VisualPtr tunnel = _scene->CreateVisual();
  MeshDescriptor tunnelDescriptor;
  tunnelDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "tunnel.dae");
  tunnelDescriptor.mesh =
      meshManager->Load(tunnelDescriptor.meshName);
  MeshPtr tunnelMesh= _scene->CreateMesh(tunnelDescriptor);
  tunnel->AddGeometry(tunnelMesh);
  tunnel->SetLocalRotation(0, 0, -1.57);
  tunnel->SetLocalPosition(0.0, 0, 0);
  root->AddChild(tunnel);

  VisualPtr entrance = _scene->CreateVisual();
  MeshDescriptor entranceDescriptor;
  entranceDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "entrance.dae");
  entranceDescriptor.mesh =
      meshManager->Load(entranceDescriptor.meshName);
  MeshPtr entranceMesh= _scene->CreateMesh(entranceDescriptor);
  entrance->AddGeometry(entranceMesh);
  entrance->SetLocalRotation(0, 0, -1.57);
  entrance->SetLocalPosition(-0, 0, 0);
  root->AddChild(entrance);

  VisualPtr parkingLot = _scene->CreateVisual();
  MeshDescriptor parkingLotDescriptor;
  parkingLotDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "parking_lot.dae");
  parkingLotDescriptor.mesh =
      meshManager->Load(parkingLotDescriptor.meshName);
  MeshPtr parkingLotMesh= _scene->CreateMesh(parkingLotDescriptor);
  parkingLot->AddGeometry(parkingLotMesh);
  parkingLot->SetLocalRotation(0, 0, -1.57);
  parkingLot->SetLocalPosition(-0, 0, 0);
  root->AddChild(parkingLot);

  VisualPtr drone = _scene->CreateVisual();
  MeshDescriptor droneDescriptor;
  droneDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "drone.dae");
  droneDescriptor.mesh =
      meshManager->Load(droneDescriptor.meshName);
  MeshPtr droneMesh= _scene->CreateMesh(droneDescriptor);
  drone->AddGeometry(droneMesh);
  drone->SetLocalRotation(0, 0, -1.57);
  drone->SetLocalPosition(0.0, 0, 0);
  root->AddChild(drone);

  VisualPtr robots01 = _scene->CreateVisual();
  MeshDescriptor robots01Descriptor;
  robots01Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_01.dae");
  robots01Descriptor.mesh =
      meshManager->Load(robots01Descriptor.meshName);
  MeshPtr robots01Mesh= _scene->CreateMesh(robots01Descriptor);
  robots01->AddGeometry(robots01Mesh);
  robots01->SetLocalRotation(0, 0, -1.57);
  robots01->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots01);

  VisualPtr robots02 = _scene->CreateVisual();
  MeshDescriptor robots02Descriptor;
  robots02Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_02.dae");
  robots02Descriptor.mesh =
      meshManager->Load(robots02Descriptor.meshName);
  MeshPtr robots02Mesh= _scene->CreateMesh(robots02Descriptor);
  robots02->AddGeometry(robots02Mesh);
  robots02->SetLocalRotation(0, 0, -1.57);
  robots02->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots02);

  VisualPtr robots03 = _scene->CreateVisual();
  MeshDescriptor robots03Descriptor;
  robots03Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_03.dae");
  robots03Descriptor.mesh =
      meshManager->Load(robots03Descriptor.meshName);
  MeshPtr robots03Mesh= _scene->CreateMesh(robots03Descriptor);
  robots03->AddGeometry(robots03Mesh);
  robots03->SetLocalRotation(0, 0, -1.57);
  robots03->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots03);

  VisualPtr robots04 = _scene->CreateVisual();
  MeshDescriptor robots04Descriptor;
  robots04Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_04.dae");
  robots04Descriptor.mesh =
      meshManager->Load(robots04Descriptor.meshName);
  MeshPtr robots04Mesh= _scene->CreateMesh(robots04Descriptor);
  robots04->AddGeometry(robots04Mesh);
  robots04->SetLocalRotation(0, 0, -1.57);
  robots04->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots04);

  VisualPtr robots05 = _scene->CreateVisual();
  MeshDescriptor robots05Descriptor;
  robots05Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_05.dae");
  robots05Descriptor.mesh =
      meshManager->Load(robots05Descriptor.meshName);
  MeshPtr robots05Mesh= _scene->CreateMesh(robots05Descriptor);
  robots05->AddGeometry(robots05Mesh);
  robots05->SetLocalRotation(0, 0, -1.57);
  robots05->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots05);

  VisualPtr robots06 = _scene->CreateVisual();
  MeshDescriptor robots06Descriptor;
  robots06Descriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "robots_06.dae");
  robots06Descriptor.mesh =
      meshManager->Load(robots06Descriptor.meshName);
  MeshPtr robots06Mesh= _scene->CreateMesh(robots06Descriptor);
  robots06->AddGeometry(robots06Mesh);
  robots06->SetLocalRotation(0, 0, -1.57);
  robots06->SetLocalPosition(0.0, 0, 0);
  root->AddChild(robots06);

  VisualPtr spotlight = _scene->CreateVisual();
  MeshDescriptor spotlightDescriptor;
  spotlightDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "spotlight.dae");
  spotlightDescriptor.mesh =
      meshManager->Load(spotlightDescriptor.meshName);
  MeshPtr spotlightMesh= _scene->CreateMesh(spotlightDescriptor);
  spotlight->AddGeometry(spotlightMesh);
  spotlight->SetLocalRotation(0, 0, -1.57);
  spotlight->SetLocalPosition(0.0, 0, 0);
  root->AddChild(spotlight);

/*
  VisualPtr tunnelStraight = _scene->CreateVisual();
  MeshDescriptor tunnelStraightDescriptor;
  tunnelStraightDescriptor.meshName =
      common::joinPaths(RESOURCE_PATH, "tunnel_straight.dae");
  tunnelStraightDescriptor.mesh =
      meshManager->Load(tunnelStraightDescriptor.meshName);
  MeshPtr tunnelStraightMesh = _scene->CreateMesh(tunnelStraightDescriptor);
  tunnelStraight->AddGeometry(tunnelStraightMesh);
  tunnelStraight->SetLocalRotation(0, 0, 1.57);
  tunnelStraight->SetLocalPosition(0, 0, 0);
  root->AddChild(tunnelStraight);
*/

/*
//  mesh->SetLocalRotation(0, 0, 1.57);
  VisualPtr mesh = _scene->CreateVisual();
  mesh->SetLocalPosition(3, 0, 0);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "tunnel_90_deg.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);
//  mesh->SetLocalRotation(0, 0, 1.57);
  mesh->SetLocalRotation(0, 0, 3.14);
  mesh->SetLocalPosition(10.0, 0, 0);
  root->AddChild(mesh);
*/


  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight("sun");
  light0->SetDirection(0.5, 0.6, -0.3);
  light0->SetDiffuseColor(1.0, 1.0, 0.8);
  light0->SetSpecularColor(0.8, 0.8, 0.5);
  root->AddChild(light0);

  // create points light
  PointLightPtr pLight = _scene->CreatePointLight();
  pLight->SetLocalPosition(0, 0, 3);
  pLight->SetDiffuseColor(0.8, 0.8, 0.6);
  pLight->SetSpecularColor(0.1, 0.1, 0.1);
  pLight->SetAttenuationRange(10);
  pLight->SetAttenuationConstant(0.5);
  pLight->SetAttenuationQuadratic(0.05);
  root->AddChild(pLight);

  PointLightPtr pLight2 = _scene->CreatePointLight();
  pLight2->SetLocalPosition(-8, 0, 3);
  pLight2->SetDiffuseColor(0.8, 0.8, 0.6);
  pLight2->SetSpecularColor(0.1, 0.1, 0.1);
  pLight2->SetAttenuationRange(10);
  pLight2->SetAttenuationConstant(0.5);
  pLight2->SetAttenuationQuadratic(0.03);
  root->AddChild(pLight2);

  // create spot lights on robots
  SpotLightPtr sLight01 = _scene->CreateSpotLight();
  sLight01->SetDiffuseColor(1.0, 1.0, 0.8);
  sLight01->SetSpecularColor(0.8, 0.8, 0.6);
  sLight01->SetLocalPosition(-12.71, 1.56, 0.182);
  sLight01->SetDirection(1, -1, 0);
  sLight01->SetAttenuationRange(10);
  sLight01->SetAttenuationConstant(0.8);
  sLight01->SetAttenuationQuadratic(0.01);
  sLight01->SetOuterAngle(math::Angle(1.0));
  sLight01->SetFalloff(1.0);
  root->AddChild(sLight01);

  SpotLightPtr sLight02 = _scene->CreateSpotLight();
  sLight02->SetDiffuseColor(1.0, 1.0, 0.8);
  sLight02->SetSpecularColor(0.8, 0.8, 0.6);
  sLight02->SetLocalPosition(-8.79, -0.65, 0.29);
  sLight02->SetDirection(1, 0.1, 0);
  sLight02->SetAttenuationRange(10);
  sLight02->SetAttenuationConstant(0.8);
  sLight02->SetAttenuationQuadratic(0.01);
  sLight02->SetOuterAngle(math::Angle(1.0));
  sLight02->SetFalloff(1.0);
  root->AddChild(sLight02);

  SpotLightPtr sLight03 = _scene->CreateSpotLight();
  sLight03->SetDiffuseColor(1.0, 1.0, 0.4);
  sLight03->SetSpecularColor(0.8, 0.8, 0.3);
  sLight03->SetLocalPosition(14.52, -2.81, 0.29);
  sLight03->SetDirection(1, -0.35, 0);
  sLight03->SetAttenuationRange(10);
  sLight03->SetAttenuationConstant(0.3);
  sLight03->SetAttenuationQuadratic(0.01);
  sLight03->SetOuterAngle(math::Angle(1.3));
  sLight03->SetFalloff(1.0);
  root->AddChild(sLight03);

  SpotLightPtr sLight04 = _scene->CreateSpotLight();
  sLight04->SetDiffuseColor(1.0, 1.0, 0.4);
  sLight04->SetSpecularColor(0.8, 0.8, 0.3);
  sLight04->SetLocalPosition(19.168, -2.66, 0.29);
  sLight04->SetDirection(1, -1.0, 0);
  sLight04->SetAttenuationRange(10);
  sLight04->SetAttenuationConstant(0.5);
  sLight04->SetAttenuationQuadratic(0.01);
  sLight04->SetOuterAngle(math::Angle(1.0));
  sLight04->SetFalloff(1.0);
  root->AddChild(sLight04);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(-15.2, 0.0, 2.0);
  camera->SetLocalRotation(0.0, 0.2, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(M_PI / 2);
  root->AddChild(camera);
}

//////////////////////////////////////////////////
CameraPtr createCamera(const std::string &_engineName)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName);
  if (!engine)
  {
    std::cout << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return CameraPtr();
  }
  ScenePtr scene = engine->CreateScene("scene");
  buildScene(scene);

  // return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<Camera>(sensor);
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  glutInit(&_argc, _argv);

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  engineNames.push_back("ogre2");
  for (auto engineName : engineNames)
  {
    try
    {
      CameraPtr camera = createCamera(engineName);
      if (camera)
      {
        cameras.push_back(camera);
      }
    }
    catch (...)
    {
      // std::cout << ex.what() << std::endl;
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
  run(cameras);
  return 0;
}
