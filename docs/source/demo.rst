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

.. Cartographer documentation master file, created by
   sphinx-quickstart on Fri Jul  8 10:41:33 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

=====================
Cartographer ROS Demo
=====================

First, follow the installation instructions in the :ref:`getting-started` section.

Now that Cartographer and Cartographer's ROS integration are installed, download the example bag:

* `2D backpack collection of the Deutches Museum
  <https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag>`_

Finally, use ``roslaunch`` to bring up the demo:

  .. code-block:: bash

    roslaunch cartographer_ros demo_2d.launch bag_filename:=<PATH_TO_BAG>

The launch file will bring up ``roscore`` and ``rviz`` automatically.
