# tod_vehicle_config

This package contains the config files of the teleoperated vehicles. To create a new config, create a new directory in [vehicle_config/](https://gitlab.lrz.de/teleoperiertes_fahren/tod_vehicle_interface/-/blob/develop/tod_vehicle_config/vehicle_config/), and follow the conventions regarding file, parameter and directory names.

Building the package, and thus providing the config files to the other packages of the software stack, is done in a typical ROS manner (via CMakeLists.txt and package.xml).
