# tod_simulation

## Quick Start

### Start Simulator
1. Download Simulator from: https://www.lgsvlsimulator.com/ (Release 2020.6)
2. Extract folder and start `simulator` executable -> Click "Open Browser"
3. Go to "Simulations" Tab
4. Choose "BorregasAve (With Autoware)" and click Play-Button on BottomRight
5. Vehicle should apper in Simulator Window -> click Play-Button in Simulator Window

### Start tod-Bridge
1. Open `vehicle.launch` in package `tod_launch`
2. Set `LGSVL` to `true`
3. `catkin build && source devel/setup.zsh && roslaunch tod_launch both.launch`
4. Go to **OperatorManagerWidget**-Gui
    - set both IP-Addresses to `127.0.0.1`
    - Click _Connect_
    - Click _Start_
5. Shift Gears (Select **virtual Input-Device** and Press "T")
6. Start Driving

## Configurations
_For more details visit: https://www.lgsvlsimulator.com/docs/maps-tab/_

### Change Vehicle/Map
- In the Browser Window of the Simulator (Tab **Vehicles**) different Vehicles are available.
- Go to the Settings of a specific Vehicle and Check if **Bridge Type** _ROS_ is selected.
- For Adjusting Sensor Configuration see: **Adjust Sensor Configuration**
- Go to Tap **Simulations**
- Choose a Existing Simlation and Click the _Edit_ or add a new simulation by clicking _Add new_
- In **Map & Vehicles** you can choose one of the existing Maps and Vehicles.
- For **ROS connection** type `localhost:9090`
- Click _Submit_ and start the simulation

### Adjust Sensor Configuration
- In the Browser Window of the Simulator (Tab **Vehicles**) different Vehicles are available.
- Go to the Settings of a specific Vehicle and edit _Sensors_
- You can also Add a new vehicle by clicking _Add_
- **Tip:** Open https://codebeautify.org/jsonviewer paste existing Configuration for better editing
- Check https://www.lgsvlsimulator.com/docs/autoware-json-example/ for examples

## Performance Improvement
For getting high framerates of 3 cameras (>= 25fps):

- Remove Lidar from sensor configuration
- Edit Simulation:
    - Under **Map & Vehicles** uncheck _Interactive Mode_
    - Under **General** check _Run simulation in Headless Mode_ 

