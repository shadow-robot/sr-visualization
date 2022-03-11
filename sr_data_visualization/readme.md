# sr_data_visualization

This is a package to graphically display data coming out of the Dexterous Hand. 

In each tab, you can find information about:

- Joint states (position, effort, velocity)
- Control loops (setpoint, input, dinput/dt, output, error)
- Motor stats (Strain Gauge Left, Strain Gauge Right, Measured PWM, Measured Current, Measured Voltage, Measured Effort, Temperature, Unfiltered position, Unfiltered force, Last Commanded Effort, Encoder Position)
- Palm extras (Accelerometer, Gyro-meter, Analog inputs)

The radio buttons let you choose specific data to show or you can choose “All” to see several graphs being displayed at the same time.

The check buttons next to each graph name allows you to show the graphs you select in larger detail by checking the boxes of the graphs you want to see and clicking “Show Selected”. To return to the full graph view click “Reset”.

This plugin supports a hand connected and a recorded ROS bag. Currently only 1 hand at a time is support - in case of two hands connected, the plugin will populate its plots for the first detected hand.

## How to use it


The gui can be started via roslaunch:

```
roslaunch sr_data_visualization data_visualizer.launch 
```
or as an rqt plugin:

```
rqt
```

and go to Plugins -> Shadow Robot -> Dexterous Hand Data Visualizer.


## Requirement

To be able to use this gui you must have installed pyqwt:

```
sudo apt install python3-qwt
```
