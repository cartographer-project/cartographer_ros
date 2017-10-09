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

#ifndef CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_
#define CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "OgreManualObject.h"
#include "OgreMaterial.h"
#include "OgreQuaternion.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "OgreTexture.h"
#include "OgreVector3.h"
#include "cartographer/mapping/id.h"
#include "cartographer_ros/submap.h"

namespace cartographer_rviz {

Ogre::Vector3 ToOgre(const Eigen::Vector3d& v);
Ogre::Quaternion ToOgre(const Eigen::Quaterniond& q);

// A class containing the Ogre code to visualize a slice texture of a submap.
// Member functions are expected to be called from the Ogre thread.
class OgreSlice {
 public:
  // Attaches a node visualizing the submap 'id' to the 'submap_node' which is
  // expected to represent the submap frame.
  OgreSlice(const ::cartographer::mapping::SubmapId& id,
            Ogre::SceneManager* const scene_manager,
            Ogre::SceneNode* const submap_node);
  ~OgreSlice();

  OgreSlice(const OgreSlice&) = delete;
  OgreSlice& operator=(const OgreSlice&) = delete;

  // Updates the texture and pose of the submap using new data from
  // 'submap_texture'.
  void Update(const ::cartographer_ros::SubmapTexture& submap_texture);

  // Changes the opacity of the submap to 'alpha'.
  void SetAlpha(float alpha);

 private:
  const ::cartographer::mapping::SubmapId id_;
  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const submap_node_;
  Ogre::SceneNode* const slice_node_;
  Ogre::ManualObject* const manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_
