# tod_simulation

## Quick Start

### Start Simulator
1. Download Simulator from: https://www.svlsimulator.com/ (Release >=2021.2.2)
2. Create a new Sensor Configuration for Lexus2016RXHybrid
3. Copy the content of `config/sensors.json` to the Sensor Configuration (Button: `{...}`). This Sensor Configuration works with the provided vehicle config in `tod_vehicle_config/vehicle_config/lgsvl`
4. Start the simulation

### Start tod-Bridge
1. Open package `tod_launch`
2. Set `vehicleID` in `both.launch` to `lgsvl`
2. Set `mode` in `vehicle.launch` to `vehicle`
3. `catkin build && source devel/setup.zsh && roslaunch tod_launch both.launch`
4. Go to **OperatorManagerWidget**-Gui
    - set both IP-Addresses to `127.0.0.1`
    - Click _Connect_
    - Click _Start_
5. Shift Gears (Select **virtual Input-Device** and Press "T")
6. Start Driving

## Configurations
_For more details visit: https://www.svlsimulator.com/docs/_

## Performance Improvement
For getting high framerates of 3 cameras:

- Remove Lidar from sensor configuration
- Edit Simulation:
    - Under **General** uncheck _Interactive Mode_
    - Under **General** check _Run simulation in Headless Mode_ 

