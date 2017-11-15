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
#ifndef IGNITION_RENDERING_OGRE_OGREUNIFORMMATERIALAPPLICATOR_HH_
#define IGNITION_RENDERING_OGRE_OGREUNIFORMMATERIALAPPLICATOR_HH_

#include <vector>

#include "ignition/rendering/ogre/OgreIncludes.hh"
#include "ignition/rendering/Util.hh"

namespace ignition
{
  namespace rendering
  {
    /// \brief Causes all objects in a scene to be rendered with the same
    /// material when rendered by a given RenderTarget.
    /// \internal
    ///
    /// On construction it registers as an Ogre::RenderTargetListener
    /// on the provided Ogre::RenderTarget.
    /// When the target is about to be rendered it adds itself as an
    /// Ogre::RenderQueueListener on the Ogre::SceneManager.
    /// Every time a queue is about to be rendered it adds itself as an
    /// Ogre::RenderQueue::RenderableListener to the queue.
    /// There it sets the technique to be used when rendering each entity.
    /// Once the scene has finished rendering it removes itself from the
    /// listeners on the scene, and sets the listener on the Ogre::RenderQueue
    /// to nullptr.
    class IGNITION_VISIBLE OgreRenderTargetMaterial :
      public Ogre::RenderTargetListener,
      public Ogre::RenderQueueListener,
      public Ogre::RenderQueue::RenderableListener
    {
      /// \brief constructor
      /// \param[in] _scene the scene manager responsible for rendering
      /// \param[in] _renderTarget the RenderTarget this should apply to
      /// \param[in] _material the material to apply to all renderables
      public: OgreRenderTargetMaterial(Ogre::SceneManager *_scene,
          Ogre::RenderTarget *_renderTarget, Ogre::Material *_material);

      /// \brief destructor
      public: ~OgreRenderTargetMaterial();

      /// \brief Callback when a render target is about to be rendered
      private: virtual void preRenderTargetUpdate(
          const Ogre::RenderTargetEvent &_evt) override;

      /// \brief Callback when a render target is finisned being rendered
      private: virtual void postRenderTargetUpdate(
          const Ogre::RenderTargetEvent &_evt) override;

      /// \brief Callback when a queue is beginning to be processed by a scene
      private: virtual void renderQueueStarted (Ogre::uint8 _groupId,
          const Ogre::String &invocation, bool &_skip) override;

      /// \brief Callback called when each object is added the render queue.
      private: virtual bool renderableQueued(Ogre::Renderable *_renderable,
          Ogre::uint8 _groupId, Ogre::ushort _priority,
          Ogre::Technique **_technique, Ogre::RenderQueue *_queue) override;

      /// \brief scene manager responsible for rendering
      private: Ogre::SceneManager *scene;

      /// \brief render target that should see a uniform material
      private: Ogre::RenderTarget *renderTarget;

      /// \brief material that should be applied to all objects
      private: Ogre::Material *material;

      /// \brief Queues to which this has been added as a listener
      private: std::vector<Ogre::RenderQueue*> renderQueues;
    };
  }
}

#endif
