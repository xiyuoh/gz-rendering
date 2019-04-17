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

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/gziface/SceneManager.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

/// \brief Test SceneManager class
class SceneManagerTest : public testing::Test,
                         public testing::WithParamInterface<const char *>
{
  /// \brief Test creating and removing entities
  public: void SceneManager(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void SceneManagerTest::SceneManager(const std::string &_renderEngine)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  // Create scene manager
  gziface::SceneManager sceneMgr;
  sceneMgr.SetScene(scene);
  EXPECT_EQ(scene, sceneMgr.Scene());

  uint64_t worldId = 0;
  // test creating model
  sdf::Model modelSdf;
  std::string modelName = "model";
  math::Pose3d modelPose(0, -1, 9, IGN_PI, 0, IGN_PI);
  modelSdf.SetName(modelName);
  modelSdf.SetPose(modelPose);
  uint64_t modelId = 10;
  sceneMgr.CreateModel(modelId, modelSdf, worldId);
  EXPECT_TRUE(sceneMgr.HasEntity(modelId));
  NodePtr model = sceneMgr.NodeById(modelId);
  EXPECT_NE(nullptr, model);
  EXPECT_EQ(modelName, model->Name());
  EXPECT_EQ(modelPose, model->LocalPose());

  // test creating link
  sdf::Link linkSdf;
  std::string linkName = "link";
  math::Pose3d linkPose(1, 2, 3, 0, 0, IGN_PI);
  linkSdf.SetName(linkName);
  linkSdf.SetPose(linkPose);
  uint64_t linkId = 11;
  sceneMgr.CreateLink(linkId, linkSdf, modelId);
  EXPECT_TRUE(sceneMgr.HasEntity(linkId));
  NodePtr link = sceneMgr.NodeById(linkId);
  EXPECT_NE(nullptr, link);
  EXPECT_EQ(modelName + "::" + linkName, link->Name());
  EXPECT_EQ(linkPose, link->LocalPose());

  // test creating visual and geom
  sdf::Visual visualSdf;
  std::string visualName = "visual";
  math::Pose3d visualPose(3, 4, 5, 0, IGN_PI, 0);
  sdf::Geometry geomSdf;
  geomSdf.SetType(sdf::GeometryType::BOX);
  sdf::Box boxSdf;
  boxSdf.SetSize(math::Vector3d(1.2, 2.3, 3.4));
  geomSdf.SetBoxShape(boxSdf);
  visualSdf.SetName(visualName);
  visualSdf.SetPose(visualPose);
  visualSdf.SetGeom(geomSdf);
  uint64_t visualId = 12;
  sceneMgr.CreateVisual(visualId, visualSdf, linkId);
  EXPECT_TRUE(sceneMgr.HasEntity(visualId));
  VisualPtr visual =
    std::dynamic_pointer_cast<Visual>(sceneMgr.NodeById(visualId));
  EXPECT_NE(nullptr, visual);
  EXPECT_EQ(modelName + "::" + linkName + "::" + visualName, visual->Name());
  EXPECT_EQ(visualPose, visual->LocalPose());
  EXPECT_EQ(1u, visual->GeometryCount());
  EXPECT_NE(nullptr, visual->GeometryByIndex(0u));

  // test creating light
  sdf::Light lightSdf;
  std::string lightName = "light";
  math::Pose3d lightPose(6, 7, 8, IGN_PI, 0, 0);
  math::Vector3d lightDir(1, 0, 1);
  sdf::LightType lightType = sdf::LightType::DIRECTIONAL;
  math::Color lightDiffuse(1, 0, 0);
  math::Color lightSpecular(0, 0.5, 0);
  bool lightCastShadows = false;
  lightSdf.SetName(lightName);
  lightSdf.SetPose(lightPose);
  lightSdf.SetDirection(lightDir);
  lightSdf.SetType(lightType);
  lightSdf.SetDiffuse(lightDiffuse);
  lightSdf.SetSpecular(lightSpecular);
  lightSdf.SetCastShadows(lightCastShadows);
  uint64_t lightId = 14;
  sceneMgr.CreateLight(lightId, lightSdf, worldId);
  EXPECT_TRUE(sceneMgr.HasEntity(lightId));
  DirectionalLightPtr light =
      std::dynamic_pointer_cast<DirectionalLight>(sceneMgr.NodeById(lightId));
  EXPECT_NE(nullptr, light);
  EXPECT_EQ(lightName, light->Name());
  EXPECT_EQ(lightPose, light->LocalPose());
  EXPECT_EQ(lightDir, light->Direction());
  EXPECT_EQ(lightDiffuse, light->DiffuseColor());
  EXPECT_EQ(lightSpecular, light->SpecularColor());
  EXPECT_EQ(lightCastShadows, light->CastShadows());

  // test adding camera
  CameraPtr camera = scene->CreateCamera("camera");
  ASSERT_NE(nullptr, camera);
  uint64_t sensorId = 15;
  sceneMgr.AddSensor(sensorId, camera->Id(), worldId);
  EXPECT_TRUE(sceneMgr.HasEntity(sensorId));
  NodePtr sensor = sceneMgr.NodeById(sensorId);
  EXPECT_EQ(camera, sensor);

  // test removing entities
  sceneMgr.RemoveEntity(sensorId);
  EXPECT_FALSE(sceneMgr.HasEntity(sensorId));
  NodePtr sensorNode = sceneMgr.NodeById(sensorId);
  EXPECT_EQ(nullptr, sensorNode);

  sceneMgr.RemoveEntity(lightId);
  EXPECT_FALSE(sceneMgr.HasEntity(lightId));
  NodePtr lightNode = sceneMgr.NodeById(lightId);
  EXPECT_EQ(nullptr, lightNode);

  sceneMgr.RemoveEntity(visualId);
  EXPECT_FALSE(sceneMgr.HasEntity(visualId));
  NodePtr visualNode = sceneMgr.NodeById(visualId);
  EXPECT_EQ(nullptr, visualNode);

  sceneMgr.RemoveEntity(linkId);
  EXPECT_FALSE(sceneMgr.HasEntity(linkId));
  NodePtr linkNode = sceneMgr.NodeById(linkId);
  EXPECT_EQ(nullptr, linkNode);

  sceneMgr.RemoveEntity(modelId);
  EXPECT_FALSE(sceneMgr.HasEntity(modelId));
  NodePtr modelNode = sceneMgr.NodeById(modelId);
  EXPECT_EQ(nullptr, modelNode);

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(SceneManagerTest, SceneManager)
{
  SceneManager(GetParam());
}

INSTANTIATE_TEST_CASE_P(SceneManager, SceneManagerTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
