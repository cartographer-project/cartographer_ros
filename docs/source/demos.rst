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

======================================
Running Cartographer ROS on a demo bag
======================================

Now that Cartographer and Cartographer's ROS integration are installed, you can
download example bags (e.g. 2D and 3D backpack collections of the
`Deutsches Museum <https://en.wikipedia.org/wiki/Deutsches_Museum>`_) to a
known location, in this case ``~/Downloads``, and use ``roslaunch`` to bring up
the demo.

The launch files will bring up ``roscore`` and ``rviz`` automatically.

.. warning:: When you want to run cartographer_ros, you might need to source your ROS environment by running ``source install_isolated/setup.bash`` first (replace bash with zsh if your shell is zsh)

Deutsches Museum
================

Download and launch the 2D backpack demo:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
    roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

Download and launch the 3D backpack demo:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
    roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

Pure localization
=================

Pure localization uses 2 different bags. The first one is used to generate the map, the second to run pure localization.

Download the 2D bags from the Deutsche Museum:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-27-12-31-41.bag

Generate the map (wait until cartographer_offline_node finishes) and then run pure localization:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
    roslaunch cartographer_ros demo_backpack_2d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag

Download the 3D bags from the Deutsche Museum:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-15-52-20.bag

Generate the map (wait until cartographer_offline_node finishes) and then run pure localization:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag
    roslaunch cartographer_ros demo_backpack_3d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b3-2016-04-05-15-52-20.bag

Static landmarks
========

  .. raw:: html

      <iframe width="560" height="315" src="https://www.youtube.com/embed/E2-OD-ycivc" frameborder="0" allowfullscreen></iframe>

  .. code-block:: bash

    # Download the landmarks example bag.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/mir/landmarks_demo_uncalibrated.bag

    # Launch the landmarks demo.
    roslaunch cartographer_mir offline_mir_100_rviz.launch bag_filename:=${HOME}/Downloads/landmarks_demo_uncalibrated.bag

Revo LDS
========

Download and launch an example bag captured from a low-cost Revo Laser Distance Sensor from Neato Robotics vacuum cleaners:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag
    roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag

PR2
===

Download and launch an example bag captured from a PR2 R&D humanoid robot from Willow Garage:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/pr2/2011-09-15-08-32-46.bag
    roslaunch cartographer_ros demo_pr2.launch bag_filename:=${HOME}/Downloads/2011-09-15-08-32-46.bag

Taurob Tracker
==============

Download and launch an example bag captured from a Taurob Tracker teleoperation robot:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/taurob_tracker/taurob_tracker_simulation.bag
    roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
