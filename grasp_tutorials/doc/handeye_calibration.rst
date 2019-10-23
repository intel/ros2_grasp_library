Hand-eye Calibration
=====================

System Requirement
----------------------
Ubuntu Linux 18.04 on 64-bit

Dependencies
---------------
`handeye_target_detection <https://github.intel.com/otc-rse/handeye_target_detection.git>`_ is used
to get the 3D pose of a calibration board. Run following commands to install the repo:

::

  cd <catkin_workspace>/src

  git clone -b master https://github.intel.com/otc-rse/handeye_target_detection.git

  cd ..

  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

  catkin build

`handeye_dashboard <https://github.intel.com/otc-rse/handeye_dashboard.git>`_ is
a ROS rqt UI for users to make hand-eye calibration operations.
Run following commands to install the repo:

::

  cd <catkin_workspace>/src

  git clone -b master https://github.com/crigroup/criutils

  git clone -b master https://github.com/crigroup/handeye.git

  git clone -b master https://github.com/crigroup/baldor.git

  git clone -b master https://github.intel.com/otc-rse/handeye_dashboard.git

  cd ..

  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

  catkin build

.. note:: A new MoveIt handeye calibration plugin can be expected in future, which is
          still under development.
