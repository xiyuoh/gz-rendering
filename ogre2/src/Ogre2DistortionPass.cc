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


#include <gz/common/Console.hh>

#include "gz/rendering/RenderPassSystem.hh"
#include "gz/rendering/ogre2/Ogre2Includes.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2DistortionPass.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"

/// \brief Private data for the Ogre2DistotionPass class
class gz::rendering::Ogre2DistortionPassPrivate
{
  /// \brief Scale applied to distorted image.
  public: gz::math::Vector2d distortionScale = {1.0, 1.0};

  /// \brief True if the distorted image will be cropped to remove the
  /// black pixels at the corners of the image.
  public: bool distortionCrop = true;

  /// \brief Pointer to the Distortion ogre material
  public: Ogre::Material *distortionMat = nullptr;

  /// \brief Pointer to the Distortion ogre texture
  public: Ogre::TextureGpu *distortionTex = nullptr;

  /// \brief Mapping of distorted to undistorted normalized pixels
  public: std::vector<gz::math::Vector2d> distortionMap;

  /// \brief Width of distortion texture map
  public: unsigned int distortionTexWidth = 0u;

  /// \brief Height of distortion texture map
  public: unsigned int distortionTexHeight = 0u;

  /// \brief Raw buffer of distortion data
  public: float *distortionBuffer = nullptr;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2DistortionPass::Ogre2DistortionPass() :
    dataPtr(std::make_unique<Ogre2DistortionPassPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2DistortionPass::~Ogre2DistortionPass()
{
}

//////////////////////////////////////////////////
void Ogre2DistortionPass::PreRender()
{
  if (!this->dataPtr->distortionMat)
    return;

  if (!this->enabled)
    return;

  Ogre::Pass *pass =
    this->dataPtr->distortionMat->getTechnique(0)->getPass(0);
  Ogre::GpuProgramParametersSharedPtr psParams =
    pass->getFragmentProgramParameters();
  psParams->setNamedConstant("scale",
    Ogre::Vector3(
      1.0 / this->dataPtr->distortionScale.X(),
      1.0 / this->dataPtr->distortionScale.Y(),
      1.0));
}

//////////////////////////////////////////////////
void Ogre2DistortionPass::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    gzerr << "No camera set for applying Distortion Pass" << std::endl;
    return;
  }

  // If no distortion is required, immediately return.
  if (gz::math::equal(this->k1, 0.0) &&
      gz::math::equal(this->k2, 0.0) &&
      gz::math::equal(this->k3, 0.0) &&
      gz::math::equal(this->p1, 0.0) &&
      gz::math::equal(this->p2, 0.0))
  {
    return;
  }

  //-------------------------------------------------------------------------------------------
  // Do some calculation here
  //-------------------------------------------------------------------------------------------
  // float viewportWidth = static_cast<float>(this->ogreCamera->
  //     getLastViewport()->getActualWidth());
  // float viewportHeight = static_cast<float>(this->ogreCamera->
  //     getLastViewport()->getActualHeight());
  // TODO(XY): fetch width and height properly from RenderTarget
  float viewportWidth = 320.0;
  float viewportHeight = 240.0;
  std::cout << "------------- did i manage to get the RenderTarget->width: " << viewportWidth << std::endl;
  std::cout << "------------- did i manage to get the RenderTarget->height: " << viewportHeight << std::endl;

  // seems to work best with a square distortion map texture
  unsigned int texSize = static_cast<unsigned int>(viewportHeight >
      viewportWidth ? viewportHeight : viewportWidth);
  // calculate focal length from largest fov
  const double fov = viewportHeight > viewportWidth ?
      this->ogreCamera->getFOVy().valueRadians() :
      (this->ogreCamera->getFOVy().valueRadians() *
      this->ogreCamera->getAspectRatio());
  const double focalLength = texSize / (2 * tan(fov / 2));
  this->dataPtr->distortionTexWidth = texSize;
  this->dataPtr->distortionTexHeight = texSize;
  unsigned int imageSize =
      this->dataPtr->distortionTexWidth * this->dataPtr->distortionTexHeight;
  double colStepSize = 1.0 / this->dataPtr->distortionTexWidth;
  double rowStepSize = 1.0 / this->dataPtr->distortionTexHeight;

  // Half step-size vector to add to the value being placed in distortion map.
  // Necessary for compositor to correctly interpolate pixel values.
  const auto halfTexelSize =
      0.5 * gz::math::Vector2d(rowStepSize, colStepSize);

  // initialize distortion map
  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
  {
    this->dataPtr->distortionMap[i] = -1;
  }

  gz::math::Vector2d distortionCenterCoordinates(
      this->lensCenter.X() * this->dataPtr->distortionTexWidth,
      this->lensCenter.Y() * this->dataPtr->distortionTexWidth);

  // declare variables before the loop
  const auto unsetPixelVector =  gz::math::Vector2d(-1, -1);
  gz::math::Vector2d normalizedLocation,
      distortedLocation,
      newDistortedCoordinates,
      currDistortedCoordinates;
  unsigned int distortedIdx,
      distortedCol,
      distortedRow;
  double normalizedColLocation, normalizedRowLocation;

  // fill the distortion map
  for (unsigned int mapRow = 0; mapRow < this->dataPtr->distortionTexHeight;
    ++mapRow)
  {
    normalizedRowLocation = mapRow*rowStepSize;
    for (unsigned int mapCol = 0; mapCol < this->dataPtr->distortionTexWidth;
      ++mapCol)
    {
      normalizedColLocation = mapCol*colStepSize;

      normalizedLocation[0] = normalizedColLocation;
      normalizedLocation[1] = normalizedRowLocation;

      distortedLocation = this->Distort(
          normalizedLocation,
          this->lensCenter,
          this->k1, this->k2, this->k3,
          this->p1, this->p2,
          this->dataPtr->distortionTexWidth,
          focalLength);

      // compute the index in the distortion map
      distortedCol = static_cast<unsigned int>(round(distortedLocation.X() *
        this->dataPtr->distortionTexWidth));
      distortedRow = static_cast<unsigned int>(round(distortedLocation.Y() *
        this->dataPtr->distortionTexHeight));

      // Note that the following makes sure that, for significant distortions,
      // there is not a problem where the distorted image seems to fold over
      // itself. This is accomplished by favoring pixels closer to the center
      // of distortion, and this change applies to both the legacy and
      // nonlegacy distortion modes.

      // Make sure the distorted pixel is within the texture dimensions
      if (distortedCol < this->dataPtr->distortionTexWidth &&
          distortedRow < this->dataPtr->distortionTexHeight)
      {
        distortedIdx = distortedRow * this->dataPtr->distortionTexWidth +
          distortedCol;

        // check if the index has already been set
        if (this->dataPtr->distortionMap[distortedIdx] != unsetPixelVector)
        {
          // grab current coordinates that map to this destination
          currDistortedCoordinates =
            this->dataPtr->distortionMap[distortedIdx] *
            this->dataPtr->distortionTexWidth;

          // grab new coordinates to map to
          newDistortedCoordinates[0] = mapCol;
          newDistortedCoordinates[1] = mapRow;

          // use the new mapping if it is closer to the center of the distortion
          if (newDistortedCoordinates.Distance(distortionCenterCoordinates) <
              currDistortedCoordinates.Distance(distortionCenterCoordinates))
          {
            this->dataPtr->distortionMap[distortedIdx] = normalizedLocation +
              halfTexelSize;
          }
        }
        else
        {
          this->dataPtr->distortionMap[distortedIdx] = normalizedLocation +
            halfTexelSize;
        }
      }
      // else: mapping is outside of the image bounds.
      // This is expected and normal to ensure
      // no black borders; carry on
    }
  }

  //-------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------

  static int distortionNodeCounter = 0;

  // Create distortion pass compositor
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  std::string nodeDefName = "DistortionPassNodeNode_"
        + std::to_string(distortionNodeCounter);

  if (ogreCompMgr->hasNodeDefinition(nodeDefName))
    return;

  // The Distortion material is defined in script (distortion.material).
  // clone the material
  std::string matName = "Distortion";
  Ogre::MaterialPtr ogreMat =
      Ogre::MaterialManager::getSingleton().getByName(matName);
  if (!ogreMat)
  {
    gzerr << "Distortion material not found: '" << matName << "'"
          << std::endl;
    return;
  }
  if (!ogreMat->isLoaded())
    ogreMat->load();
  std::string materialName = matName + "_" +
      std::to_string(distortionNodeCounter);
  this->dataPtr->distortionMat = ogreMat->clone(materialName).get();

  // create the compositor node definition

  // We need to programmatically create the compositor because we need to
  // configure it to use the cloned distortion material created earlier.
  // The compositor workspace definition is equivalent to the following
  // ogre compositor script:
  // compositor_node DistortionPass
  // {
  //   // render texture input from previous render pass
  //   in 0 rt_input
  //   // render texture output to be passed to next render pass
  //   in 1 rt_output
  //
  //   // Only one target pass is needed
  //   // rt_input is used as input to this pass and result is stored
  //   // in rt_output
  //
  //   target distortionMap
  //   {
  //     pass render_scene
  //     {
  //     }
  //   }
  //   target rt_output
  //   {
  //     pass render_quad
  //     {
  //       material Distortion // Use copy instead of original
  //       input 0 rt_input
  //       input 1 distortionMap
  //     }
  //   }
  //   // pass the result to the next render pass
  //   out 0 rt_output
  //   // pass the rt_input render texture to the next render pass
  //   // where the teture is reused to store its result
  //   out 1 rt_input
  // }

  this->ogreCompositorNodeDefName = nodeDefName;
  distortionNodeCounter++;

  Ogre::CompositorNodeDef *nodeDef =
      ogreCompMgr->addNodeDefinition(nodeDefName);

  // Input texture
  nodeDef->addTextureSourceName("rt_input", 0,
      Ogre::TextureDefinitionBase::TEXTURE_INPUT);
  Ogre::TextureDefinitionBase::TextureDefinition *distortionTexDef =
      nodeDef->addTextureDefinition("distortionMap");
  distortionTexDef->textureType = Ogre::TextureTypes::Type2D;
  distortionTexDef->width = 0; // set to distortion tex width
  distortionTexDef->height = 0; // set to distortion tex height
  distortionTexDef->depthOrSlices = 1;
  distortionTexDef->numMipmaps = 0;
  distortionTexDef->widthFactor = 1;
  distortionTexDef->heightFactor = 1;
  distortionTexDef->format = Ogre::PFG_RGB32_FLOAT;
  distortionTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
  distortionTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
  distortionTexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;

  nodeDef->addTextureSourceName("rt_output", 0,
      Ogre::TextureDefinitionBase::TEXTURE_INPUT);

  Ogre::RenderTargetViewDef *rtv =
    nodeDef->addRenderTextureView("distortionMap");
  rtv->setForTextureDefinition("distortionMap", distortionTexDef);

  nodeDef->setNumTargetPass(2);
  // distortionMap target
  Ogre::CompositorTargetDef *distortionTargetDef =
      nodeDef->addTargetPass("distortionMap");
  distortionTargetDef->setNumPasses(1);
  {
    // scene pass
    Ogre::CompositorPassSceneDef *passScene =
        static_cast<Ogre::CompositorPassSceneDef *>(
        distortionTargetDef->addPass(Ogre::PASS_SCENE));
    passScene->setAllLoadActions(Ogre::LoadAction::Clear);
    // passScene->setAllClearColours(Ogre::ColourValue(0, 0, 0));
  }

  // rt_input target
  Ogre::CompositorTargetDef *inputTargetDef =
      nodeDef->addTargetPass("rt_output");
  inputTargetDef->setNumPasses(1);
  {
    // quad pass
    Ogre::CompositorPassQuadDef *passQuad =
        static_cast<Ogre::CompositorPassQuadDef *>(
        inputTargetDef->addPass(Ogre::PASS_QUAD));
    passQuad->mMaterialName = materialName;
    passQuad->addQuadTextureSource(0, "rt_input");
    passQuad->addQuadTextureSource(0, "distortionMap");
    passQuad->mFrustumCorners =
        Ogre::CompositorPassQuadDef::VIEW_SPACE_CORNERS;
  }

  nodeDef->mapOutputChannel(0, "rt_output");
  nodeDef->mapOutputChannel(1, "rt_input");

  // create distortion map texture
  Ogre::TextureGpuManager *textureMgr =
    ogreRoot->getRenderSystem()->getTextureGpuManager();
  std::string texName = this->ogreCamera->getName() + "_distortionTex";
  this->dataPtr->distortionTex =
    textureMgr->createOrRetrieveTexture(texName,
      "General",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::RenderToTexture,
      Ogre::TextureTypes::Type2D);

  this->dataPtr->distortionTex->setResolution(
    this->dataPtr->distortionTexWidth,
    this->dataPtr->distortionTexHeight);
  this->dataPtr->distortionTex->setNumMipmaps(1u);
  this->dataPtr->distortionTex->setPixelFormat(Ogre::PFG_RGB32_FLOAT);

  this->dataPtr->distortionTex->scheduleTransitionTo(
    Ogre::GpuResidency::Resident);

  // TODO(XY): Find a way to update pixel buffer
  unsigned int width = this->dataPtr->distortionTexWidth;
  unsigned int height = this->dataPtr->distortionTexHeight;

  gz::rendering::PixelFormat format = PF_FLOAT32_RGB;
  unsigned int rawChannelCount = PixelUtil::ChannelCount(format);
  unsigned int bytesPerChannel = PixelUtil::BytesPerChannel(format);
  int rawLen = width * height * rawChannelCount;

  if (!this->dataPtr->distortionBuffer)
  {
    this->dataPtr->distortionBuffer = new float[rawLen];
  }

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->distortionTex, 0u, 0u);
  Ogre::TextureBox box = image.getData(0u);
  float *bufferTmp = static_cast<float *>(box.data);

  // Raw buffer
  for (unsigned int i = 0; i < height; ++i)
  {
    unsigned int rawDataRowIdx = i * box.bytesPerRow / bytesPerChannel;
    unsigned int rowIdx = i * width * rawChannelCount;
    memcpy(&this->dataPtr->distortionBuffer[rowIdx], &bufferTmp[rawDataRowIdx],
        width * rawChannelCount * bytesPerChannel);
  }

}

//////////////////////////////////////////////////
gz::math::Vector2d Ogre2DistortionPass::Distort(
    const gz::math::Vector2d &_in,
    const gz::math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2, unsigned int _width, double _f)
{
  // apply Brown's distortion model, see
  // http://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction

  gz::math::Vector2d normalized2d = (_in - _center) * (_width / _f);
  gz::math::Vector3d normalized(normalized2d.X(), normalized2d.Y(), 0);
  double rSq = normalized.X() * normalized.X() +
               normalized.Y() * normalized.Y();

  // radial
  gz::math::Vector3d dist = normalized * (1.0 +
      _k1 * rSq +
      _k2 * rSq * rSq +
      _k3 * rSq * rSq * rSq);

  // tangential
  dist.X() += _p2 * (rSq + 2 * (normalized.X()*normalized.X())) +
      2 * _p1 * normalized.X() * normalized.Y();
  dist.Y() += _p1 * (rSq + 2 * (normalized.Y()*normalized.Y())) +
      2 * _p2 * normalized.X() * normalized.Y();

  return ((_center * _width) +
    gz::math::Vector2d(dist.X(), dist.Y()) *_f) / _width;
}

//////////////////////////////////////////////////
gz::math::Vector2d
    Ogre2DistortionPass::DistortionMapValueClamped(
    int _x, int _y) const
{
  if (_x < 0 || _x >= static_cast<int>(this->dataPtr->distortionTexWidth) ||
      _y < 0 || _y >= static_cast<int>(this->dataPtr->distortionTexHeight))
  {
    return gz::math::Vector2d(-1, -1);
  }
  gz::math::Vector2d res =
      this->dataPtr->distortionMap[_y * this->dataPtr->distortionTexWidth + _x];
  return res;
}

//////////////////////////////////////////////////
void Ogre2DistortionPass::CalculateAndApplyDistortionScale()
{
  //
}

GZ_RENDERING_REGISTER_RENDER_PASS(Ogre2DistortionPass, DistortionPass)
