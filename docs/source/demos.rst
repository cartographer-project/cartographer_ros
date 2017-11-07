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

=====
Demos
=====

Pure localization
=================

  .. code-block:: bash

    # Pure localization demo in 2D: We use 2 different 2D bags from the Deutsche
    # Museum. The first one is used to generate the map, the second to run
    # pure localization.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-27-12-31-41.bag
    # Generate the map: Run the next command, wait until cartographer_offline_node finishes.
    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
    # Run pure localization:
    roslaunch cartographer_ros demo_backpack_2d_localization.launch \
       map_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag

    # Pure localization demo in 3D: We use 2 different 3D bags from the Deutsche
    # Museum. The first one is used to generate the map, the second to run
    # pure localization.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-15-52-20.bag
    # Generate the map: Run the next command, wait until cartographer_offline_node finishes.
    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag
    # Run pure localization:
    roslaunch cartographer_ros demo_backpack_3d_localization.launch \
       map_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b3-2016-04-05-15-52-20.bag

Revo LDS
========

  .. code-block:: bash

    # Download the Revo LDS example bag.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag

    # Launch the Revo LDS demo.
    roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag

PR2
===

  .. code-block:: bash

    # Download the PR2 example bag.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/pr2/2011-09-15-08-32-46.bag

    # Launch the PR2 demo.
    roslaunch cartographer_ros demo_pr2.launch bag_filename:=${HOME}/Downloads/2011-09-15-08-32-46.bag

Taurob Tracker
==============

  .. code-block:: bash

    # Download the Taurob Tracker example bag.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/taurob_tracker/taurob_tracker_simulation.bag

    # Launch the Taurob Tracker demo.
    roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
