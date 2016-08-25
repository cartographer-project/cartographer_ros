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

=====================
Cartographer ROS Demo
=====================

First, follow the installation instructions in the :ref:`getting-started` section.

Now that Cartographer and Cartographer's ROS integration are installed,
download the example bag, a 2D backpack collection of the `Deutsches Museum
<https://en.wikipedia.org/wiki/Deutsches_Museum>`_, to a known location, in
this case ``~/Downloads``, and use ``roslaunch`` to bring up the demo:

  .. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
    roslaunch cartographer_ros demo_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

The launch file will bring up ``roscore`` and ``rviz`` automatically.
