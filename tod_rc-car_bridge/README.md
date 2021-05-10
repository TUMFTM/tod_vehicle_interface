# tod_rc-car_bridge
Establishes a communication with F1TENTH type of RC-Car. However, the package
can be easily modified to support other types of remotely-controlled
vehicles, too.

## Nodes
The package consists of the following set of nodes:

### CommandToRCCar
**Publication:** `/vesc/low_level/ackermann_cmd_mux/input/navigation` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))  
**Subscription:** `/Vehicle/VehicleBridge/primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))

### DataToRCCar
**Publication:** `/Vehicle/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))  

#### Subscriptions:
* `/hedge_imu` ([sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))
* `/vesc/commands/motor/speed` ([std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html))
* `/vesc/commands/servo/position` ([std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html))
* `/vesc/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html))

Additionally, there are 2 nodes that are republishing topics:
* `RepublishFront`: Republishing `/scan` to `/Vehicle/Lidar/laser_model/scan`
* `RepublishOdom`: Republishing `/vesc/odom` to `/Vehicle/VehicleBridge/odometry`
