.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

Algorithm walkthrough for tuning
================================

Cartographer is a complex system and tuning it requires a good understanding of its inner working.
This page tries to give an intuitive overview of the different subsystems used by Cartographer along with their configuration values.
If you are interested in more than an introduction to Cartographer, you should refer to the Cartographer paper.
It only describes the 2D SLAM but it defines rigourously most of the concepts described here.
Those concepts generally apply to 3D as well.

W. Hess, D. Kohler, H. Rapp, and D. Andor,
`Real-Time Loop Closure in 2D LIDAR SLAM`_, in
*Robotics and Automation (ICRA), 2016 IEEE International Conference on*.
IEEE, 2016. pp. 1271–1278.

.. _Real-Time Loop Closure in 2D LIDAR SLAM: https://research.google.com/pubs/pub45466.html

Overview
--------

.. image:: https://raw.githubusercontent.com/googlecartographer/cartographer/master/docs/source/high_level_system_overview.png
     :target: https://github.com/googlecartographer/cartographer/blob/master/docs/source/high_level_system_overview.png

Cartographer can be seen as two separate, but related subsystems.
The first one is **local SLAM** (sometimes also called **frontend** or local trajectory builder).
Its job is to build a succession of **submaps**.
Each submap is meant to be locally consistent but we accept that local SLAM drifts over time.
Most of the local SLAM options can be found in `install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua`_ for 2D and `install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua`_ for 3D. (for the rest of this page we will refer to `TRAJECTORY_BUILDER_nD` for the common options)

.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua: https://github.com/googlecartographer/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_2d.lua
.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua: https://github.com/googlecartographer/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_3d.lua

The other subsystem is **global SLAM** (sometimes called the **backend**).
It runs in background threads and its main job is to find **loop closure constraints**.
It does that by scan-matching **scans** (gathered in **nodes**) against submaps.
It also incorporates other sensor data to get a higher level view and identify the most consistent global solution.
In 3D, it also tries to find the direction of gravity.
Most of its options can be found in `install_isolated/share/cartographer/configuration_files/pose_graph.lua`_

.. _install_isolated/share/cartographer/configuration_files/pose_graph.lua: https://github.com/googlecartographer/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/pose_graph.lua

On a higher abstraction, the job of local SLAM is to generate good submaps and the job of global SLAM is to tie them most consistently together.

Input
-----

Range finding sensors (for example: LIDARs) provide depth information in multiple directions.
However, some of the measurements are irrelevant for SLAM.
If the sensor is partially covered with dust or if it is directed towards a part of the robot, some of the measured distance can be considered as noise for SLAM.
On the other hand, some of the furthest measurements can also come from undesired sources (reflection, sensor noise) and are irrelevant for SLAM as well.
To tackle those issue, Cartographer starts by applying a bandpass filter and only keeps range values between a certain min and max range.
Those min and max values should be chosen according to the specifications of your robot and sensors.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.min_range
    TRAJECTORY_BUILDER_nD.max_range

.. note::

    In 2D, Cartographer replaces ranges further than max_range by ``TRAJECTORY_BUILDER_2D.missing_data_ray_length``. It also provides a ``max_z`` and ``min_z`` values to filter 3D point clouds into a 2D cut.

.. note::

    In Cartographer configuration files, every distance is defined in meters

Distances are measured over a certain period of time, while the robot is actually moving.
However, distances are delivered by sensors "in batch" in large ROS messages.
Each of the messages' timestamp can be considered independently by Cartographer to take into account deformations caused by the robot's motion.
The more often Cartographer gets measurements, the better it becomes at unwarping the measurements to assemble a single coherent scan that could have been captured instantly.
It is therefore strongly encouraged to provide as many range data (ROS messages) by scan (a set of range data that can be matched against another scan) as possible.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.num_accumulated_range_data

Range data is typically measured from a single point on the robot but in multiple angles. 
This means that close surfaces (for instance the road) are very often hit and provide lots of points.
On the opposite, far objects are less often hit and offer less points.
In order to reduce the computational weight of points handling, we usually need to subsample point clouds.
However, a simple random sampling would remove points from areas where we already have a low density of measurements and the high-density areas would still have more points than needed.
To address that density problem, we can use a voxel filter that downsamples raw points into cubes of a constant size and only keeps the centroid of each cube.

A small cube size will result in a more dense data representation, causing more computations.
A large cube size will result in a data loss but will be much quicker.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.voxel_filter_size

After having applied a fixed-size voxel filter, Cartographer also applies an **adaptive voxel filter**.
This filter tries to determine the optimal voxel size (under a max length) to achieve a target number of points.
In 3D, two adaptive voxel filters are used to generate a high resolution and a low resolution point clouds, their usage will be clarified in :ref:`local-slam`.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points

An Inertial Measurement Unit can be an useful source of information for SLAM because it provides an accurate direction of gravity (hence, of the ground) and a noisy but good overall indication of the robot's rotation.
In order to filter the IMU noise, gravity is observed over a certain amount of time.
If you use 2D SLAM, range data can be handled in real-time without an additional source of information so you can choose whether you'd like Cartographer to use an IMU or not.
With 3D SLAM, you need to provide an IMU because it is used as an initial guess for the orientation of the scans, greatly reducing the complexity of scan matching.


.. code-block:: lua

    TRAJECTORY_BUILDER_2D.use_imu_data
    TRAJECTORY_BUILDER_nD.imu_gravity_time_constant

.. note::

    In Cartographer configuration files, every time value is defined in seconds

.. _local-slam:

Local SLAM
----------

Once a scan has been assembled and filtered from multiple range data, it is ready for the local SLAM algorithm.
Local SLAM inserts a new scan into its current submap construction by **scan matching** using an initial guess from the **pose extrapolator**.
The idea behind the pose extrapolator is to use sensor data of other sensors besides the range finder to predict where the next scan should be inserted into the submap.

Two scan matching strategies are available: 

- The ``CeresScanMatcher`` takes the initial guess as prior and finds the best spot where the scan match fits the submap.
  It does this by interpolating the submap and sub-pixel aligning the scan.
  This is fast, but cannot fix errors that are significantly larger than the resolution of the submaps.
  If your sensor setup and timing is reasonable, using only the ``CeresScanMatcher`` is usually the best choice to make.
- The ``RealTimeCorrelativeScanMatcher`` can be enabled if you do not have other sensors or you do not trust them.
  It uses an approach similar to how scans are matched against submaps in loop closure (described later), but instead it matches against the current submap.
  The best match is then used as prior for the ``CeresScanMatcher``.
  This scan matcher is very expensive and will essentially override any signal from other sensors but the range finder, but it is robust in feature rich environments.

Either way, the ``CeresScanMatcher`` can be configured to give a certain weight to each of its input.
The weight is a measure of trust into your data, this can be seen as a static covariance.
The unit of weight parameters are dimensionless quantities and can't be compared between each others.
The bigger the weight of a source of data is, the more emphasis Cartographer will put on this source of data when doing scan matching.
Sources of data include occupied space (points from the scan), translation and rotation from the pose extrapolator (or ``RealTimeCorrelativeScanMatcher``)

.. code-block:: lua

    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight
    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0
    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight

.. note::

    In 3D, the ``occupied_space_weight_0`` and ``occupied_space_weight_1`` parameters are related, respectively, to the high resolution and low resolution filtered point clouds.

The ``CeresScanMatcher`` gets its name from `Ceres Solver`_, a library developed at Google to solve non-linear least squares problems.
The scan matching problem is modelled as the minimization of such a problem with the **motion** (a transformation matrix) between two scans being a parameter to determine.
Ceres optimizes the motion using a descent algorithm for a given number of iterations.
Ceres can be configured to adapt the convergence speed to your own needs.

.. _Ceres Solver: http://ceres-solver.org/

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads

The ``RealTimeCorrelativeScanMatcher`` can be toggled depending on the trust you have in your sensors.
It works by searching for similar scans in a **search window** which is defined by a maximum distance radius and a maximum angle radius.
When performing scan matching with scans found in this window, a different weight can be chosen for the translational and rotational components.
You can play with those weight if, for example, you know that your robot doesn't rotate a lot.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.use_online_correlative_scan_matching
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.linear_search_window
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.angular_search_window
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.translation_delta_cost_weight
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.rotation_delta_cost_weight

To avoid inserting too many scans per submaps, once a motion between two scans is found by the scan matcher, it goes through a **motion filter**.
A scan is dropped if the motion that led to it is not considered as significant enough.
A scan is inserted into the current submap only if its motion is above a certain distance, angle or time threshold.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.motion_filter.max_time_seconds
    TRAJECTORY_BUILDER_nD.motion_filter.max_distance_meters
    TRAJECTORY_BUILDER_nD.motion_filter.max_angle_radians

A submap is considered as complete when the local SLAM has received a given amount of range data.
Local SLAM drifts over time, global SLAM is used to fix this drift.
Submaps must be small enough so that the drift inside them is below the resolution, so that they are locally correct.
On the other hand, they should be large enough to be distinct for loop closure to work properly.

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.submaps.num_range_data

Submaps can store their range data in a couple of different data structures:
The most widely used representation is called probability grids.
However, in 2D, one can also choose to use Truncated Signed Distance Fields (TSDF).

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type

Probability grids cut out space into a 2D or 3D table where each cell has a fixed size and contains the odds of being obstructed.
Odds are updated according to "*hits*" (where the range data is measured) and "*misses*" (the free space between the sensor and the measured points).
Both *hits* and *misses* can have a different weight in occupancy probability calculations giving more or less trust to occupied or free space measurements.

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability
    TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability
    TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability
    TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability

In 2D, only one probability grid per submap is stored.
In 3D, for scan matching performance reasons, two *hybrid* probability grids are used.
(the term "hybrid" only refers to an internal tree-like data representation and is abstracted to the user)

- a low resolution hybrid grid for far measurements
- a high resolution hybrid grid for close measurements

Scan matching starts by aligning far points of the low resolution point cloud with the low resolution hybrid grid and then refines the pose by aligning the close high resolution points with the high resolution hybrid grid.

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution
    TRAJECTORY_BUILDER_3D.submaps.high_resolution
    TRAJECTORY_BUILDER_3D.submaps.low_resolution
    TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range
    TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range

.. note::

    Cartographer ROS provides an RViz plugin to visualize submaps. You can select the submaps you want to see from their number. In 3D, RViz only shows 2D projections of the 3D hybrid probability grids (in grayscale). Options are made available in RViz's left pane to switch between the low and high resolution hybrid grids visualization.

**TODO**: *Documenting TSDF configuration*

Global SLAM
-----------

While the local SLAM generates its succession of submaps, a global optimization (usually refered to as "*the optimization problem*" or "*sparse pose adjustment*") task runs in background.
Its role is to re-arrange submaps between each other so that they form a coherent global map.
For instance, this optimization is in charge of altering the currently built trajectory to properly align submaps with regards to loop closures.

The optimization is run in batches once a certain number of trajectory nodes was inserted. Depending on how frequently you need to run it, you can tune the size of these batches.

.. code-block:: lua

    POSE_GRAPH.optimize_every_n_nodes

.. note::

    Setting POSE_GRAPH.optimize_every_n_nodes to 0 is a handy way to disable global SLAM and concentrate on the behavior of local SLAM. This is usually one of the first thing to do to tune Cartographer.

The global SLAM is a kind of "*GraphSLAM*", it is essentially a pose graph optimization which works by building **constraints** between **nodes** and submaps and then optimizing the resulting constraints graph.
Constraints can intuitively be thought of as little ropes tying all nodes together.
The sparse pose adjustement fastens those ropes altogether.
The resulting net is called the "*pose graph*".

.. note::

    Constraints can be visualized in RViz, it is very handy to tune global SLAM. One can also toggle ``POSE_GRAPH.constraint_builder.log_matches`` to get regular reports of the constraints builder formatted as histograms.

- Non-global constraints (also known as inter submaps constraints) are built automatically between nodes that are closely following each other on a trajectory.
  Intuitively, those "*non-global ropes*" keep the local structure of the trajectory coherent.
- Global constraints (also referred to as loop closure constraints or intra submaps contraints) are regularly searched between a new submap and previous nodes that are considered "*close enough*" in space (part of a certain **search window**) and a strong fit (a good match when running scan matching).
  Intuitively, those "*global ropes*" introduce knots in the structure and firmly bring two strands closer.

.. code-block:: lua

    POSE_GRAPH.constraint_builder.max_constraint_distance
    POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window
    POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_xy_search_window
    POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_z_search_window
    POSE_GRAPH.fast_correlative_scan_matcher*.angular_search_window

.. note::

    In practice, global constraints can do more than finding loop closures on a single trajectory. They can also align different trajectories recorded by multiple robots but we will keep this usage and the parameters related to "global localization" out of the scope of this document.

To limit the amount of constraints (and computations), Cartographer only considers a subsampled set of all close nodes for constraints building.
This is controlled by a sampling ratio constant.
Sampling too few nodes could result in missed constraints and ineffective loop closures.
Sampling too many nodes would slow the global SLAM down and prevent real-time loop closures.

.. code-block:: lua

    POSE_GRAPH.constraint_builder.sampling_ratio

When a node and a submap are considered for constraint building, they go through a first scan matcher called the ``FastCorrelativeScanMatcher``.
This scan matcher has been specifically designed for Cartographer and makes real-time loop closures scan matching possible.
The ``FastCorrelativeScanMatcher`` relies on a "*Branch and bound*" mechanism to work at different grid resolutions and efficiently eliminate incorrect matchings.
This mechanism is extensively presented in the Cartographer paper presented earlier in this document.
It works on an exploration tree whose depth can be controlled.

.. code-block:: lua

    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth
    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth

Once the ``FastCorrelativeScanMatcher`` has a good enough proposal (above a minimum score of matching), it is then fed into a Ceres Scan Matcher to refine the pose.

.. code-block:: lua

    POSE_GRAPH.constraint_builder.min_score
    POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d
    POSE_GRAPH.constraint_builder.ceres_scan_matcher

When Cartographer runs *the optimization problem*, Ceres is used to rearrange submaps according to multiple *residuals*.
Residuals are calculated using weighted cost functions.
The global optimization has cost functions to take into account plenty of data sources: the global (loop closure) constraints, the non-global (matcher) constraints, the IMU acceleration and rotation measurements, the local SLAM rough pose estimations, an odometry source or a fixed frame (such as a GPS system).
The weights and Ceres options can be configured as described in the :ref:`local-slam` section.

.. code-block:: lua

    POSE_GRAPH.constraint_builder.loop_closure_translation_weight
    POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
    POSE_GRAPH.matcher_translation_weight
    POSE_GRAPH.matcher_rotation_weight
    POSE_GRAPH.optimization_problem.*_weight
    POSE_GRAPH.optimization_problem.ceres_solver_options

.. note::

    One can find useful information about the residuals used in the optimization problem by toggling ``POSE_GRAPH.max_num_final_iterations``

As part of its IMU residual, the optimization problem gives some flexibility to the IMU pose and, by default, Ceres is free to optimize the extrinsic calibration between your IMU and tracking frame.
If you don't trust your IMU pose, the results of Ceres' global optimization can be logged and used to improve your extrinsic calibration.
If Ceres doesn't optimize your IMU pose correctly and you trust your extrinsic calibration enough, you can make this pose constant.

.. code-block:: lua

    POSE_GRAPH.optimization_problem.log_solver_summary
    POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d

In residuals, the influence of outliers is handled by a **Huber loss** function configured with a certain a Huber scale.
The bigger the Huber scale, `the higher is the impact`_ of (potential) outliers.

.. _the higher is the impact: https://github.com/ceres-solver/ceres-solver/blob/0d3a84fce553c9f7aab331f0895fa7b1856ef5ee/include/ceres/loss_function.h#L172

.. code-block:: lua

    POSE_GRAPH.optimization_problem.huber_scale

Once the trajectory is finished, Cartographer runs a new global optimization with, typically, a lot more iterations than previous global optimizations.
This is done to polish the final result of Cartographer and usually does not need to be real-time so a large number of iterations is often a right choice.

.. code-block:: lua

    POSE_GRAPH.max_num_final_iterations
