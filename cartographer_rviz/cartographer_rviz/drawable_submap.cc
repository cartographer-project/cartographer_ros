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

#include "cartographer_rviz/drawable_submap.h"

#include <chrono>
#include <future>
#include <sstream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "OgreGpuProgramParams.h"
#include "OgreImage.h"
#include "cartographer/common/port.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "eigen_conversions/eigen_msg.h"
#include "ros/ros.h"

namespace cartographer_rviz {

namespace {

constexpr std::chrono::milliseconds kMinQueryDelayInMs(250);
constexpr char kSubmapTexturePrefix[] = "SubmapTexture";
constexpr char kManualObjectPrefix[] = "ManualObjectSubmap";
constexpr char kSubmapMaterialPrefix[] = "SubmapMaterial";
constexpr char kSubmapSourceMaterialName[] = "cartographer_ros/Submap";

// Distance before which the submap will be shown at full opacity, and distance
// over which the submap will then fade out.
constexpr double kFadeOutStartDistanceInMeters = 1.;
constexpr double kFadeOutDistanceInMeters = 2.;
constexpr float kAlphaUpdateThreshold = 0.2f;

std::string GetSubmapIdentifier(const int trajectory_id,
                                const int submap_index) {
  return std::to_string(trajectory_id) + "-" + std::to_string(submap_index);
}

}  // namespace

DrawableSubmap::DrawableSubmap(const int trajectory_id, const int submap_index,
                               Ogre::SceneManager* const scene_manager)
    : trajectory_id_(trajectory_id),
      submap_index_(submap_index),
      scene_manager_(scene_manager),
      scene_node_(scene_manager->getRootSceneNode()->createChildSceneNode()),
      manual_object_(scene_manager->createManualObject(
          kManualObjectPrefix +
          GetSubmapIdentifier(trajectory_id, submap_index))),
      last_query_timestamp_(0) {
  material_ = Ogre::MaterialManager::getSingleton().getByName(
      kSubmapSourceMaterialName);
  material_ =
      material_->clone(kSubmapMaterialPrefix +
                       GetSubmapIdentifier(trajectory_id_, submap_index));
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthBias(-1.f, 0.f);
  material_->setDepthWriteEnabled(false);
  scene_node_->attachObject(manual_object_);
  connect(this, SIGNAL(RequestSucceeded()), this, SLOT(UpdateSceneNode()));
}

DrawableSubmap::~DrawableSubmap() {
  if (QueryInProgress()) {
    rpc_request_future_.wait();
  }
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
}

void DrawableSubmap::Update(
    const ::std_msgs::Header& header,
    const ::cartographer_ros_msgs::SubmapEntry& metadata,
    ::rviz::FrameManager* const frame_manager) {
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!frame_manager->transform(header, metadata.pose, position, orientation)) {
    // We don't know where we would display the texture, so we stop here.
    return;
  }
  ::cartographer::common::MutexLocker locker(&mutex_);
  position_ = position;
  orientation_ = orientation;
  submap_z_ = metadata.pose.position.z;
  metadata_version_ = metadata.submap_version;
  if (texture_version_ != -1) {
    // We have to update the transform since we are already displaying a texture
    // for this submap.
    UpdateTransform();
  }
}

bool DrawableSubmap::MaybeFetchTexture(ros::ServiceClient* const client) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  const bool newer_version_available = texture_version_ < metadata_version_;
  const std::chrono::milliseconds now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  const bool recently_queried =
      last_query_timestamp_ + kMinQueryDelayInMs > now;
  if (!newer_version_available || recently_queried || query_in_progress_) {
    return false;
  }
  query_in_progress_ = true;
  last_query_timestamp_ = now;
  rpc_request_future_ = std::async(std::launch::async, [this, client]() {
    ::cartographer_ros_msgs::SubmapQuery srv;
    srv.request.trajectory_id = trajectory_id_;
    srv.request.submap_index = submap_index_;
    if (client->call(srv)) {
      // We emit a signal to update in the right thread, and pass via the
      // 'response_' member to simplify the signal-slot connection slightly.
      ::cartographer::common::MutexLocker locker(&mutex_);
      response_ = srv.response;
      Q_EMIT RequestSucceeded();
    } else {
      ::cartographer::common::MutexLocker locker(&mutex_);
      query_in_progress_ = false;
    }
  });
  return true;
}

bool DrawableSubmap::QueryInProgress() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  return query_in_progress_;
}

void DrawableSubmap::SetAlpha(const double current_tracking_z) {
  const double distance_z = std::abs(submap_z_ - current_tracking_z);
  const double fade_distance =
      std::max(distance_z - kFadeOutStartDistanceInMeters, 0.);
  const float alpha = static_cast<float>(
      std::max(0., 1. - fade_distance / kFadeOutDistanceInMeters));

  const Ogre::GpuProgramParametersSharedPtr parameters =
      material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  parameters->setNamedConstant("u_alpha", UpdateAlpha(alpha));
}

void DrawableSubmap::UpdateSceneNode() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  texture_version_ = response_.submap_version;
  std::string compressed_cells(response_.cells.begin(), response_.cells.end());
  std::string cells;
  ::cartographer::common::FastGunzipString(compressed_cells, &cells);
  tf::poseMsgToEigen(response_.slice_pose, slice_pose_);
  UpdateTransform();
  query_in_progress_ = false;
  // The call to Ogre's loadRawData below does not work with an RG texture,
  // therefore we create an RGB one whose blue channel is always 0.
  std::vector<char> rgb;
  for (int i = 0; i < response_.height; ++i) {
    for (int j = 0; j < response_.width; ++j) {
      auto r = cells[(i * response_.width + j) * 2];
      auto g = cells[(i * response_.width + j) * 2 + 1];
      rgb.push_back(r);
      rgb.push_back(g);
      rgb.push_back(0.);
    }
  }

  manual_object_->clear();
  const float metric_width = response_.resolution * response_.width;
  const float metric_height = response_.resolution * response_.height;
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
      kSubmapTexturePrefix + GetSubmapIdentifier(trajectory_id_, submap_index_);
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, response_.width, response_.height, Ogre::PF_BYTE_RGB,
      Ogre::TEX_TYPE_2D, 0);

  Ogre::Pass* const pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const texture_unit =
      pass->getNumTextureUnitStates() > 0 ? pass->getTextureUnitState(0)
                                          : pass->createTextureUnitState();

  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);
}

void DrawableSubmap::UpdateTransform() {
  const Eigen::Quaterniond quaternion(slice_pose_.rotation());
  const Ogre::Quaternion slice_rotation(quaternion.w(), quaternion.x(),
                                        quaternion.y(), quaternion.z());
  const Ogre::Vector3 slice_translation(slice_pose_.translation().x(),
                                        slice_pose_.translation().y(),
                                        slice_pose_.translation().z());
  scene_node_->setPosition(orientation_ * slice_translation + position_);
  scene_node_->setOrientation(orientation_ * slice_rotation);
}

float DrawableSubmap::UpdateAlpha(const float target_alpha) {
  if (std::abs(target_alpha - current_alpha_) > kAlphaUpdateThreshold ||
      target_alpha == 0.f || target_alpha == 1.f) {
    current_alpha_ = target_alpha;
  }
  return current_alpha_;
}

}  // namespace cartographer_rviz
