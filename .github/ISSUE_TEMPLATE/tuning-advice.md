---
name: Help request for tuning
about: Describe tuning problems with your custom Cartographer setup to request help from the community.

---

To improve the chances that other Cartographer users can help you,
here are some guidelines for describing your tuning problem:

1. check the tuning guide on the documentation page and previous issues -
   some problems might have been solved already by other people.

2. run `rosbag_validate` which does some checks on your sensor data. This
   tool often finds issues that can explain poor performance and must be fixed
   at recording time. Please post the full output of the tool into a
   GitHub Gist at https://gist.github.com/, then link it in the issue even if
   it does not report anything. You can run the tool like this:

   rosrun cartographer_ros cartographer_rosbag_validate -bag_filename <bag filename>

3. post a link to a Git repository containing a branch of
   `cartographer_ros` containing all the configuration, launch, and URDF files
   required to reproduce your issue.
4. post a link to a bag file we can use to reproduce your issue. Put it on
   Google Drive, Dropbox, any webserver or wherever it is publicly
   downloadable.
5. remove this boilerplate text before submitting your issue.

PLEASE NOTE: tuning issues are not necessarily handled by the maintainers and
can be closed after a period of inactivity.
