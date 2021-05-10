# tod_vehicle_sim

The package simulates the vehicle given the vehicle configuration data (see `tod_vehicle_config`). The node consumes control commands,
while generating odometry and vehicle data streams.

## VehicleSim

**Publications**:
* `/Vehicle/VehicleBridge/odometry` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* `/Vehicle/VehicleBridge/odometry_rear` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* `/Vehicle/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))

**Subscriptions**:
* `/Vehicle/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))
* `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
* `/Vehicle/VehicleBridge/primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
