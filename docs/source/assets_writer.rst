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

.. cartographer SHA: 30f7de1a325d6604c780f2f74d9a345ec369d12d
.. cartographer_ros SHA: 44459e18102305745c56f92549b87d8e91f434fe

Assets writer
=============

The purpose of SLAM is to compute the trajectory of a single sensor through a metric space.
On a higher level, the input of SLAM is sensor data, its output is the best estimate of the trajectory up to this point in time.
To be real-time and efficient, Cartographer throws most of the sensor data away immediately.

The trajectory alone is rarely of interest.
But once the best trajectory is established, the full sensor data can be used to derive and visualize information about its surroundings.

Cartographer provides the assets writer for this.
Its inputs are

1. the original sensor data fed to SLAM in a ROS bag file,
2. the cartographer state, which is contained in the ``.pbstream`` file that SLAM creates,
3. the sensor extrinsics (i.e. TF data from the bag or a URDF),
4. and a pipeline configuration, which is defined in a ``.lua`` file.

The assets writer runs through the sensor data in batches with a known trajectory.
It can be used to color, filter and export SLAM point cloud data in a variety of formats.
For more information on what the asset writer can be used for, refer to the examples below below and the header files in `cartographer/io`_.

Sample usage
------------

.. code-block:: bash

   # Download the 3D backpack example bag.
   wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-14-14-00.bag

   # Launch the 3D offline demo.
   roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

Watch the output on the commandline until the offline node terminates.
It will have written ``b3-2016-04-05-14-14-00.bag.pbstream`` which represents the Cartographer state after it processed all data and finished all optimizations.
You could have gotten the same state data by running the online node and calling:

.. code-block:: bash

   # Finish the first trajectory. No further data will be accepted on it.
   rosservice call /finish_trajectory 0

   # Ask Cartographer to serialize its current state.
   rosservice call /write_state ${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream

Now we run the assets writer with the `sample configuration file`_ for the 3D backpack:

.. _sample configuration file: https://github.com/googlecartographer/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_3d.lua

.. code-block:: bash

   roslaunch cartographer_ros assets_writer_backpack_3d.launch \
      bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag \
      pose_graph_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream

All output files are prefixed by ``--output_file_prefix`` which defaults to the filename of the first bag.
For the last example, if you specify ``points.ply`` in the pipeline configuration file, this will translate to ``${HOME}/Downloads/b3-2016-04-05-14-14-00.bag_points.ply``.

Configuration
-------------

The assets writer is modeled as a pipeline.
It consists of `PointsProcessor`_\ s and `PointsBatch`_\ s  flow through it.
Data flows from the first processor to the next, each has the chance to modify the ``PointsBatch`` before passing it on.

.. _PointsProcessor: https://github.com/googlecartographer/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/points_processor.h
.. _PointsBatch: https://github.com/googlecartographer/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/points_batch.h

For example the `assets_writer_backpack_3d.lua`_ uses ``min_max_range_filter`` to remove points that are either too close or too far from the sensor.
After this, it writes X-Rays, then recolors the ``PointsBatch``\ s depending on the sensor frame ids and writes another set of X-Rays using these new colors.

.. _assets_writer_backpack_3d.lua: https://github.com/googlecartographer/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_3d.lua

The individual ``PointsProcessor``\ s are all in the `cartographer/io`_ sub-directory and documented in their individual header files.

.. _cartographer/io: https://github.com/googlecartographer/cartographer/tree/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io

First-person visualization of point clouds
------------------------------------------

Generating a fly through of points is a two step approach:
First, write a PLY file with the points you want to visualize, then use `point_cloud_viewer`_.

.. _point_cloud_viewer: https://github.com/googlecartographer/point_cloud_viewer

The first step is usually accomplished by using IntensityToColorPointsProcessor_ to give the points a non-white color, then writing them to a PLY using PlyWritingPointsProcessor_.
An example is in `assets_writer_backpack_2d.lua`_.

.. _IntensityToColorPointsProcessor: https://github.com/googlecartographer/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/intensity_to_color_points_processor.cc
.. _PlyWritingPointsProcessor: https://github.com/googlecartographer/cartographer/blob/30f7de1a325d6604c780f2f74d9a345ec369d12d/cartographer/io/ply_writing_points_processor.h
.. _assets_writer_backpack_2d.lua: https://github.com/googlecartographer/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_2d.lua

Once you have the PLY, follow the README of `point_cloud_viewer`_ to generate an on-disk octree data structure which can be viewed by one of the viewers in the same repo.

.. _point_cloud_viewer: https://github.com/googlecartographer/point_cloud_viewer
