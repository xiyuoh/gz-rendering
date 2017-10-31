/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/rendering/ogre/OgreCamera.hh"
#include "ignition/rendering/ogre/OgreConversions.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"
#include "ignition/rendering/ogre/OgreMaterial.hh"
#include "ignition/rendering/ogre/OgreRenderTarget.hh"
#include "ignition/rendering/ogre/OgreScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreCamera::OgreCamera()
{
}

//////////////////////////////////////////////////
OgreCamera::~OgreCamera()
{
}

//////////////////////////////////////////////////
void OgreCamera::SetHFOV(const math::Angle &_angle)
{
  BaseCamera::SetHFOV(_angle);
  double hfov = _angle.Radian();
  double vfov = 2.0 * atan(tan(hfov / 2.0) / this->aspect);
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));
}

//////////////////////////////////////////////////
double OgreCamera::AspectRatio() const
{
  return this->ogreCamera->getAspectRatio();
}

//////////////////////////////////////////////////
void OgreCamera::SetAspectRatio(const double _ratio)
{
  BaseCamera::SetAspectRatio(_ratio);
  return this->ogreCamera->setAspectRatio(_ratio);
}

//////////////////////////////////////////////////
unsigned int OgreCamera::AntiAliasing() const
{
  return this->renderTexture->AntiAliasing();
}

//////////////////////////////////////////////////
void OgreCamera::SetAntiAliasing(const unsigned int _aa)
{
  BaseCamera::SetAntiAliasing(_aa);
  this->renderTexture->SetAntiAliasing(_aa);
}

//////////////////////////////////////////////////
math::Color OgreCamera::BackgroundColor() const
{
  return this->renderTexture->BackgroundColor();
}

//////////////////////////////////////////////////
void OgreCamera::SetBackgroundColor(const math::Color &_color)
{
  this->renderTexture->SetBackgroundColor(_color);
}

//////////////////////////////////////////////////
void OgreCamera::SetGlobalMaterial(const MaterialPtr &_material)
{
  this->globalMaterial = _material->Clone();

  if (this->globalMaterial)
  {
    OgreMaterialPtr ogreMaterial= std::dynamic_pointer_cast<OgreMaterial>(
        this->globalMaterial);
    Ogre::MaterialPtr material = ogreMaterial->Material();
    material->load();
  }
}

//////////////////////////////////////////////////
void OgreCamera::Render()
{
  if (this->globalMaterial)
  {
    OgreMaterialPtr ogreMaterial= std::dynamic_pointer_cast<OgreMaterial>(
        this->globalMaterial);
    Ogre::MaterialPtr material = ogreMaterial->Material();
    Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
    Ogre::RenderSystem *renderSys = sceneMgr->getDestinationRenderSystem();
    Ogre::RenderTarget *target = this->renderTexture->RenderTarget();
    Ogre::Viewport *vp = target->getViewport(0);
    Ogre::Technique *technique = material->getBestTechnique();

    for (int i = 0; i < technique->getNumPasses() ; ++i)
    {
      Ogre::Pass *pass = technique->getPass(i);
      sceneMgr->_setPass(pass, true, false);

      Ogre::AutoParamDataSource autoParamDataSource;
      autoParamDataSource.setCurrentPass(pass);
      autoParamDataSource.setCurrentViewport(vp);
      autoParamDataSource.setCurrentRenderTarget(target);
      autoParamDataSource.setCurrentSceneManager(sceneMgr);
      autoParamDataSource.setCurrentCamera(this->ogreCamera, true);

      renderSys->setLightingEnabled(false);
      renderSys->_setFog(Ogre::FOG_NONE);

      pass->_updateAutoParams(&autoParamDataSource, 1);

      if (pass->hasVertexProgram())
      {
        const Ogre::GpuProgramPtr &vertexShader = pass->getVertexProgram();
        renderSys->bindGpuProgram(vertexShader->_getBindingDelegate());
      }
      if (pass->hasFragmentProgram())
      {
        const Ogre::GpuProgramPtr &fragmentShader = pass->getFragmentProgram();
        renderSys->bindGpuProgram(fragmentShader->_getBindingDelegate());
      }

      this->renderTexture->Render();
    }
  }
  else
  {
    this->renderTexture->Render();
  }
}


//////////////////////////////////////////////////
RenderTargetPtr OgreCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void OgreCamera::Init()
{
  BaseCamera::Init();
  this->CreateCamera();
  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
void OgreCamera::CreateCamera()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  this->ogreCamera = ogreSceneManager->createCamera(this->name);
  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to Gazebo coordinate system
  this->ogreCamera->yaw(Ogre::Degree(-90.0));
  this->ogreCamera->roll(Ogre::Degree(-90.0));
  this->ogreCamera->setFixedYawAxis(false);

  // TODO: provide api access
  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setRenderingDistance(0);
  this->ogreCamera->setPolygonMode(Ogre::PM_SOLID);
  this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

//////////////////////////////////////////////////
void OgreCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<OgreRenderTexture>(base);
  this->renderTexture->SetCamera(this->ogreCamera);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->renderTexture->SetBackgroundColor(this->scene->BackgroundColor());
}

//////////////////////////////////////////////////
RenderWindowPtr OgreCamera::CreateRenderWindow()
{
  RenderWindowPtr base = this->scene->CreateRenderWindow();
  OgreRenderWindowPtr renderWindow =
      std::dynamic_pointer_cast<OgreRenderWindow>(base);
  renderWindow->SetWidth(this->ImageWidth());
  renderWindow->SetHeight(this->ImageHeight());
  renderWindow->SetDevicePixelRatio(1);
  renderWindow->SetCamera(this->ogreCamera);
  renderWindow->SetBackgroundColor(this->scene->BackgroundColor());

  this->renderTexture = renderWindow;
  return base;
}

//////////////////////////////////////////////////
math::Matrix4d OgreCamera::ProjectionMatrix() const
{
  return OgreConversions::Convert(this->ogreCamera->getProjectionMatrix());
}

//////////////////////////////////////////////////
math::Matrix4d OgreCamera::ViewMatrix() const
{
  return OgreConversions::Convert(this->ogreCamera->getViewMatrix(true));
}

//////////////////////////////////////////////////
void OgreCamera::SetNearClipPlane(const double _near)
{
  // this->nearClip = _near;
  BaseCamera::SetNearClipPlane(_near);
  this->ogreCamera->setNearClipDistance(_near);
}

//////////////////////////////////////////////////
void OgreCamera::SetFarClipPlane(const double _far)
{
  BaseCamera::SetFarClipPlane(_far);
  this->ogreCamera->setFarClipDistance(_far);
}
