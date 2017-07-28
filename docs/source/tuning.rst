.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

.. cartographer SHA: ea7c39b6f078c693b92fed06d86ca501021147d9
.. cartographer_ros SHA: 44459e18102305745c56f92549b87d8e91f434fe
.. TODO(hrapp): mention insert_free_space somewhere

Tuning
======

Tuning Cartographer is unfortunately really difficult.
The system has many parameters many of which affect each other.
This tuning guide tries to explain a principled approach on concrete examples.

Two systems
-----------

Cartographer can be seen as two separate, but related systems.
The first one is local SLAM (sometimes also called frontend).
Its job is build a locally consistent set of submaps and tie them together, but it will drift over time.
Most of its options can be found in `trajectory_builder_2d.lua`_ for 2D and `trajectory_builder_3d.lua`_ for 3D.

.. _trajectory_builder_2d.lua: https://github.com/googlecartographer/cartographer/blob/ea7c39b6f078c693b92fed06d86ca501021147d9/configuration_files/trajectory_builder_2d.lua
.. _trajectory_builder_3d.lua: https://github.com/googlecartographer/cartographer/blob/ea7c39b6f078c693b92fed06d86ca501021147d9/configuration_files/trajectory_builder_3d.lua

The other system is global SLAM (sometimes called the backend).
It runs in background threads and its main job is to find loop closure constraints.
It does that by scan-matching scans against submaps.
It also incorporates other sensor data to get a higher level view and identify the most consistent global solution.
In 3D, it also tries to find the direction of gravity.
Most of its options can be found in `sparse_pose_graph.lua`_

.. _sparse_pose_graph.lua: https://github.com/googlecartographer/cartographer/blob/ea7c39b6f078c693b92fed06d86ca501021147d9/configuration_files/sparse_pose_graph.lua

On a higher abstraction, the job of local SLAM is to generate good submaps and the job of global SLAM is to tie them most consistently together.

Tuning local SLAM
-----------------

For this example we'll start at ``cartographer`` commit `ea7c39b`_ and ``cartographer_ros`` commit `44459e1`_ and look at the bag ``b2-2016-04-27-12-31-41.bag`` from our test data set.

At our starting configuration, we see some slipping pretty early in the bag.
The backpack passed over a ramp in the Deutsches Museum which violates the 2D assumption of a flat floor.
It is visible in the laser scan data that contradicting information is passed to the SLAM.
But the slipping also indicates that we trust the point cloud matching too much and disregard the other sensors quite strongly.
Our aim is to improve the situation through tuning.

.. _ea7c39b: https://github.com/googlecartographer/cartographer/commit/ea7c39b6f078c693b92fed06d86ca501021147d9
.. _44459e1: https://github.com/googlecartographer/cartographer_ros/commit/44459e18102305745c56f92549b87d8e91f434fe

If we only look at this particular submap, that the error is fully contained in one submap.
We also see that over time, global SLAM figures out that something weird happened and partially corrects for it.
The broken submap is broken forever though.

.. TODO(hrapp): VIDEO

Since the problem here is slippage inside a submap, it is a local SLAM issue.
So let's turn off global SLAM to not mess with our tuning.

.. code-block:: lua

   SPARSE_POSE_GRAPH.optimize_every_n_scans = 0

Correct size of submaps
^^^^^^^^^^^^^^^^^^^^^^^

Local SLAM drifts over time, only loop closure can fix this drift.
Submaps must be small enough so that the drift inside them is below the resolution, so that they are locally correct.
On the other hand, they should be large enough to be being distinct for loop closure to work properly.
The size of submaps is configured through ``TRAJECTORY_BUILDER_2D.submaps.num_range_data``.
Looking at the individual submaps for this example they already fit the two constraints rather well, so we assume this parameter is well tuned.

The choice of scan matchers
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The idea behind local SLAM is to use sensor data of other sensors besides the range finder to predict where the next scan should be inserted into the submap.
Then, the ``CeresScanMatcher`` takes this as prior and finds the best spot where the scan match fits the submap.
It does this by interpolating the submap and sub-pixel aligning the scan.
This is fast, but cannot fix errors that are significantly larger than the resolution of the submaps.
If your sensor setup and timing is reasonable, using only the ``CeresScanMatcher`` is usually the best choice to make.

If you do not have other sensors or you do not trust them, Cartographer also provides a ``RealTimeCorrelativeScanMatcher``.
It uses an approach similar to how scans are matched against submaps in loop closure, but instead it matches against the current submap.
The best match is then used as prior for the ``CeresScanMatcher``.
This scan matcher is very expensive and will essentially override any signal from other sensors but the range finder, but it is robust in feature rich environments.

Tuning the correlative scan matcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO

Tuning the ``CeresScanMatcher``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In our case, the scan matcher can freely move the match forward and backwards without impacting the score.
We'd like to penalize this situation by making the scan matcher pay more for deviating from the prior that it got.
The two parameters controlling this are ``TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`` and ``rotation_weight``.
The higher, the more expensive it is to move the result away from the prior, or in other words: scan matching has to generate a higher score in another position to be accepted.

For instructional purposes, let's make deviating from the prior really expensive:

.. code-block:: lua

   TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3

.. TODO(hrapp): video

This allows the optimizer to pretty liberally overwrite the scan matcher results.
This results in poses close to the prior, but inconsistent with the depth sensor and clearly broken.
Experimenting with this value yields a better result at ``2e2``.

.. TODO(hrapp): VIDEO with translation_weight = 2e2

Here, the scan matcher used rotation to still slightly mess up the result though.
Setting the ``rotation_weight`` to ``4e2`` leaves us with a reasonable result.


Verification
------------

To make sure that we did not overtune for this particular issue, we need to run the configuration against other collected data.
In this case, the new parameters did reveal slipping, for example at the beginning of ``b2-2016-04-05-14-44-52.bag``, so we had to lower the ``translation_weight`` to ``1e2``.
This setting is worse for the case we wanted to fix, but no longer slips.
Before checking them in, we normalize all weights, since they only have relative meaning.
The result of this tuning was `PR 428`_.
In general, always try to tune for a platform, not a particular bag.

.. _PR 428: https://github.com/googlecartographer/cartographer/pull/428

