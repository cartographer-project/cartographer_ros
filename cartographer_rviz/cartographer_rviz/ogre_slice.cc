/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_rviz/ogre_slice.h"

#include <string>
#include <vector>

#include "OgreGpuProgramParams.h"
#include "OgreImage.h"
#include "cartographer/common/port.h"

namespace cartographer_rviz {

namespace {

constexpr char kManualObjectPrefix[] = "ManualObjectSubmap";
constexpr char kSubmapSourceMaterialName[] = "cartographer_ros/Submap";
constexpr char kSubmapMaterialPrefix[] = "SubmapMaterial";
constexpr char kSubmapTexturePrefix[] = "SubmapTexture";

std::string GetSubmapIdentifier(
    const ::cartographer::mapping::SubmapId& submap_id) {
  return std::to_string(submap_id.trajectory_id) + "-" +
         std::to_string(submap_id.submap_index);
}

}  // namespace

Ogre::Vector3 ToOgre(const Eigen::Vector3d& v) {
  return Ogre::Vector3(v.x(), v.y(), v.z());
}

Ogre::Quaternion ToOgre(const Eigen::Quaterniond& q) {
  return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z());
}

OgreSlice::OgreSlice(const ::cartographer::mapping::SubmapId& id,
                     Ogre::SceneManager* const scene_manager,
                     Ogre::SceneNode* const submap_node)
    : id_(id),
      scene_manager_(scene_manager),
      submap_node_(submap_node),
      slice_node_(submap_node_->createChildSceneNode()),
      manual_object_(scene_manager_->createManualObject(
          kManualObjectPrefix + GetSubmapIdentifier(id))) {
  material_ = Ogre::MaterialManager::getSingleton().getByName(
      kSubmapSourceMaterialName);
  material_ =
      material_->clone(kSubmapMaterialPrefix + GetSubmapIdentifier(id_));
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthBias(-1.f, 0.f);
  material_->setDepthWriteEnabled(false);
  slice_node_->attachObject(manual_object_);
}

OgreSlice::~OgreSlice() {
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
  scene_manager_->destroySceneNode(slice_node_);
  scene_manager_->destroyManualObject(manual_object_);
}

void OgreSlice::Update(
    const ::cartographer_ros::SubmapTexture& submap_texture) {
  slice_node_->setPosition(ToOgre(submap_texture.slice_pose.translation()));
  slice_node_->setOrientation(ToOgre(submap_texture.slice_pose.rotation()));
  // The call to Ogre's loadRawData below does not work with an RG texture,
  // therefore we create an RGB one whose blue channel is always 0.
  std::vector<char> rgb;
  CHECK_EQ(submap_texture.intensity.size(), submap_texture.alpha.size());
  for (size_t i = 0; i < submap_texture.intensity.size(); ++i) {
    rgb.push_back(submap_texture.intensity[i]);
    rgb.push_back(submap_texture.alpha[i]);
    rgb.push_back(0);
  }

  manual_object_->clear();
  const float metric_width = submap_texture.resolution * submap_texture.width;
  const float metric_height = submap_texture.resolution * submap_texture.height;
  manual_object_->begin(material_->getName(),
                        Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  // Bottom left
  manual_object_->position(-metric_height, 0.0f, 0.0f);
  manual_object_->textureCoord(0.0f, 1.0f);
  // Bottom right
  manual_object_->position(-metric_height, -metric_width, 0.0f);
  manual_object_->textureCoord(1.0f, 1.0f);
  // Top left
  manual_object_->position(0.0f, 0.0f, 0.0f);
  manual_object_->textureCoord(0.0f, 0.0f);
  // Top right
  manual_object_->position(0.0f, -metric_width, 0.0f);
  manual_object_->textureCoord(1.0f, 0.0f);
  manual_object_->end();

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(rgb.data(), rgb.size()));

  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
  const std::string texture_name =
      kSubmapTexturePrefix + GetSubmapIdentifier(id_);
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, submap_texture.width, submap_texture.height,
      Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D, 0);

  Ogre::Pass* const pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const texture_unit =
      pass->getNumTextureUnitStates() > 0 ? pass->getTextureUnitState(0)
                                          : pass->createTextureUnitState();

  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);
}

void OgreSlice::SetAlpha(const float alpha) {
  const Ogre::GpuProgramParametersSharedPtr parameters =
      material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  parameters->setNamedConstant("u_alpha", alpha);
}

}  // namespace cartographer_rviz
