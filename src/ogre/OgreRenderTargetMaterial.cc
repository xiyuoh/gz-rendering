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

#include "ignition/rendering/ogre/OgreRenderTargetMaterial.hh"

using namespace ignition::rendering;


//////////////////////////////////////////////////
OgreRenderTargetMaterial::OgreRenderTargetMaterial(
    Ogre::SceneManager *_scene, Ogre::RenderTarget *_renderTarget,
    Ogre::Material *_material):
  scene(_scene), renderTarget(_renderTarget), material(_material)
{
  this->renderTarget->addListener(this);
}

//////////////////////////////////////////////////
OgreRenderTargetMaterial::~OgreRenderTargetMaterial()
{
  this->renderTarget->removeListener(this);
}

//////////////////////////////////////////////////
void OgreRenderTargetMaterial::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  this->scene->addRenderQueueListener(this);
}

//////////////////////////////////////////////////
void OgreRenderTargetMaterial::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  // Unset the listener on the render queues
  // one might think renderQueueEnded or postRenderQueues could be used
  // instead, but both are called prior to the objects in the queue being
  // rendered :(
  for (auto *queue : this->renderQueues)
  {
    queue->setRenderableListener(nullptr);
  }
  this->renderQueues.clear();

  this->scene->removeRenderQueueListener(this);
}

//////////////////////////////////////////////////
void OgreRenderTargetMaterial::renderQueueStarted (Ogre::uint8 /*_group*/,
          const Ogre::String & /*invocation*/, bool & /*_skip*/)
{
  Ogre::RenderQueue *queue = this->scene->getRenderQueue();
  queue->setRenderableListener(this);
  this->renderQueues.push_back(queue);
}

//////////////////////////////////////////////////
bool OgreRenderTargetMaterial::renderableQueued(
    Ogre::Renderable * /*_renderable*/, Ogre::uint8 /*_group*/,
    Ogre::ushort /*_priority*/, Ogre::Technique **_technique,
    Ogre::RenderQueue * /*_queue*/)
{
  *_technique = this->material->getBestTechnique();
  return true;
}
