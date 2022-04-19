# tod_tum-q7_bridge
Reads vehicle data and writes control commands from and to Autobox of Q7 vehicle at TUM.


## Dependencies
  * C++17
  * ROS Packages: see `package.xml`
  * Other Libraries: none


## Nodes
The package consists of the following set of nodes:

### ControlCommandToAutobox
**Subscriptions:**
* `/Vehicle/VehicleBridge/primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
* `/Vehicle/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))

### SafetyDriverStatusFromAutobox
**Publication:** `/Vehicle/VehicleBridge/safety_driver_status` ([tod_msgs/SafetyDriverStatus](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SafetyDriverStatus.msg))

### VehicleDataFromAutobox
**Publications:**
* `/Vehicle/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))
* `/Vehicle/VehicleBridge/vehicle_twist` ([geometry_msgs::TwistWithCovarianceStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))
