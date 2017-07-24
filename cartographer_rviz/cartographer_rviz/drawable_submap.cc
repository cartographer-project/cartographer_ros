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
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/msg_conversion.h"
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

std::string GetSubmapIdentifier(
    const ::cartographer::mapping::SubmapId& submap_id) {
  return std::to_string(submap_id.trajectory_id) + "-" +
         std::to_string(submap_id.submap_index);
}

Ogre::Vector3 ToOgre(const Eigen::Vector3d& v) {
  return Ogre::Vector3(v.x(), v.y(), v.z());
}

Ogre::Quaternion ToOgre(const Eigen::Quaterniond& q) {
  return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z());
}

}  // namespace

DrawableSubmap::DrawableSubmap(const ::cartographer::mapping::SubmapId& id,
                               ::rviz::DisplayContext* const display_context,
                               ::rviz::Property* const submap_category,
                               const bool visible, const float pose_axes_length,
                               const float pose_axes_radius)
    : id_(id),
      display_context_(display_context),
      scene_node_(display_context->getSceneManager()
                      ->getRootSceneNode()
                      ->createChildSceneNode()),
      submap_node_(scene_node_->createChildSceneNode()),
      manual_object_(display_context->getSceneManager()->createManualObject(
          kManualObjectPrefix + GetSubmapIdentifier(id))),
      pose_axes_(display_context->getSceneManager(), scene_node_,
                 pose_axes_length, pose_axes_radius),
      last_query_timestamp_(0) {
  material_ = Ogre::MaterialManager::getSingleton().getByName(
      kSubmapSourceMaterialName);
  material_ =
      material_->clone(kSubmapMaterialPrefix + GetSubmapIdentifier(id_));
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthBias(-1.f, 0.f);
  material_->setDepthWriteEnabled(false);
  // DrawableSubmap creates and manages its visibility property object
  // (a unique_ptr is needed because the Qt parent of the visibility
  // property is the submap_category object - the BoolProperty needs
  // to be destroyed along with the DrawableSubmap)
  visibility_ = ::cartographer::common::make_unique<::rviz::BoolProperty>(
      "" /* title */, visible, "" /* description */, submap_category,
      SLOT(ToggleVisibility()), this);
  submap_node_->attachObject(manual_object_);
  scene_node_->setVisible(visible);
  connect(this, SIGNAL(RequestSucceeded()), this, SLOT(UpdateSceneNode()));
}

DrawableSubmap::~DrawableSubmap() {
  // 'query_in_progress_' must be true until the Q_EMIT has happened. Qt then
  // makes sure that 'RequestSucceeded' is not called after our destruction.
  if (QueryInProgress()) {
    rpc_request_future_.wait();
  }
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
  display_context_->getSceneManager()->destroySceneNode(submap_node_);
  display_context_->getSceneManager()->destroySceneNode(scene_node_);
  display_context_->getSceneManager()->destroyManualObject(manual_object_);
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
  metadata_version_ = metadata.submap_version;
  pose_ = ::cartographer_ros::ToRigid3d(metadata.pose);
  scene_node_->setPosition(ToOgre(pose_.translation()));
  scene_node_->setOrientation(ToOgre(pose_.rotation()));
  if (submap_texture_ != nullptr) {
    display_context_->queueRender();
  }
  visibility_->setName(
      QString("%1.%2").arg(id_.submap_index).arg(metadata_version_));
  visibility_->setDescription(
      QString("Toggle visibility of this individual submap.<br><br>"
              "Trajectory %1, submap %2, submap version %3")
          .arg(id_.trajectory_id)
          .arg(id_.submap_index)
          .arg(metadata_version_));
}

bool DrawableSubmap::MaybeFetchTexture(ros::ServiceClient* const client) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // Received metadata version can also be lower if we restarted Cartographer.
  const bool newer_version_available =
      submap_texture_ == nullptr ||
      submap_texture_->version != metadata_version_;
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
    std::unique_ptr<::cartographer_ros::SubmapTexture> submap_texture =
        ::cartographer_ros::FetchSubmapTexture(id_, client);
    ::cartographer::common::MutexLocker locker(&mutex_);
    query_in_progress_ = false;
    if (submap_texture != nullptr) {
      // We emit a signal to update in the right thread, and pass via the
      // 'submap_texture_' member to simplify the signal-slot connection
      // slightly.
      submap_texture_ = std::move(submap_texture);
      Q_EMIT RequestSucceeded();
    }
  });
  return true;
}

bool DrawableSubmap::QueryInProgress() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  return query_in_progress_;
}

void DrawableSubmap::SetAlpha(const double current_tracking_z) {
  const double distance_z =
      std::abs(pose_.translation().z() - current_tracking_z);
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
  submap_node_->setPosition(ToOgre(submap_texture_->slice_pose.translation()));
  submap_node_->setOrientation(ToOgre(submap_texture_->slice_pose.rotation()));
  // The call to Ogre's loadRawData below does not work with an RG texture,
  // therefore we create an RGB one whose blue channel is always 0.
  std::vector<char> rgb;
  for (size_t i = 0; i < submap_texture_->intensity.size(); ++i) {
    rgb.push_back(submap_texture_->intensity[i]);
    rgb.push_back(submap_texture_->alpha[i]);
    rgb.push_back(0.);
  }

  manual_object_->clear();
  const float metric_width =
      submap_texture_->resolution * submap_texture_->width;
  const float metric_height =
      submap_texture_->resolution * submap_texture_->height;
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
      pixel_stream, submap_texture_->width, submap_texture_->height,
      Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D, 0);

  Ogre::Pass* const pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const texture_unit =
      pass->getNumTextureUnitStates() > 0 ? pass->getTextureUnitState(0)
                                          : pass->createTextureUnitState();

  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);

  display_context_->queueRender();
}

float DrawableSubmap::UpdateAlpha(const float target_alpha) {
  if (std::abs(target_alpha - current_alpha_) > kAlphaUpdateThreshold ||
      target_alpha == 0.f || target_alpha == 1.f) {
    current_alpha_ = target_alpha;
  }
  return current_alpha_;
}

void DrawableSubmap::ToggleVisibility() {
  scene_node_->setVisible(visibility_->getBool());
}

}  // namespace cartographer_rviz
