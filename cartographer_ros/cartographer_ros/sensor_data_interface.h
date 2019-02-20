#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_DATA_INTERFACE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_DATA_INTERFACE_H

#include <string>

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

class SensorDataInterface {
 public:
  virtual void AddOdometryData(
      int trajectory_id, const std::string& sensor_id,
      const carto::sensor::OdometryData& odometry_data) = 0;

  virtual void AddImuData(int trajectory_id, const std::string& sensor_id,
                          const carto::sensor::ImuData& imu_data) = 0;

  virtual void AddTrajectoryNodeData(
      int trajectory_id,
      std::shared_ptr<const carto::mapping::TrajectoryNode::Data>
          trajectory_node_data) = 0;

  virtual void AddSubmap(
      int trajectory_id,
      std::shared_ptr<const carto::mapping::Submap> submap) = 0;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_DATA_INTERFACE_H