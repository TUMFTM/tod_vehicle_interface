# tod_vehicle_bridge

The main package that creates an interface to the vehicle. The vehicle can be any simulator,
1:10th car or real vehicle, as long as it provides the topics listed below.

## Dependencies

* tod_msgs
* tod_network

## Documentation

### VehicleDataReceiver (Operator)

**Publications**:

* `/Operator/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))

**Subscriptions**: None

### OdometryReceiver (Operator)

**Publications**:

* `/Operator/VehicleBridge/odometry` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

**Subscriptions**: None

### GPSReceiver (Operator)

**Publications**:

* `/Operator/VehicleBridge/gps/fix` ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))

**Subscriptions**: None

### VehicleDataSender (Vehicle)

**Publications**: None

**Subscriptions**:

* `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
* `/Vehicle/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))

### ControlCommandMultiplexer (Vehicle)

Subscribes to multiple primary_control_cmds and forwards one to the vehicle. The desired primary_control_cmd is selected depending on the selected control mode in the status_msg.  
**Publications**:

* `/Vehicle/VehicleBridge/primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))

**Subscriptions**:

* `/Vehicle/DirectControl/primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
* `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))

### OdometrySender (Vehicle)

**Publications**: None.

**Subscriptions**:

* `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
* `/Vehicle/VehicleBridge/odometry` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

### GPSSender (Vehicle)

**Publications**: None.

**Subscriptions**:

* `/Vehicle/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg))
* `/Vehicle/VehicleBridge/gps/fix` ([sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html))
