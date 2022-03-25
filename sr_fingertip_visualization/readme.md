# sr_fingertip_visualization

This is a package to graphically display data coming from the tactile sensors of the Dexterous Hand. 

There are 2 available tabs:
- **Visualizer**
- **Graphs**

As a user you can select which hands and corresponding sensors you would like to inspect by selecting the **HandID**.
Selecting a specific finger will enable or disable the refreshing. You have also the possibility to present only selected fingers by pressing **Show selected** or bring back all of the fingers to the tab by pressing **Show all**.

The **Visualizer** tab respresents the data in form of tactile points changing their colours based on the value coming from the sensors. In case of a Dexterous Hand equiped with Biotacs as tactile sensors, there is also a button which will allow you to switch the visual representation mode of the tactile points between **electrodes** or **pac** values coming from the sensor.

The **Graphs** tab respresents the data in form of plots for all of the data coming from the sensors. Ticking the corresponding checkbox for the datatype will either add or remove the plot from the graph of the finger.


## How to use it


The gui can be started via roslaunch with an optional rosbag. The rosbag will be played with the -l option (infinite loop):

```
roslaunch sr_gui_fingertip_visualization tactile_visualizer.launch rosbag_path:=<absolute_path>
```
or as an rqt plugin:

```
rqt
```

and go to Plugins -> Shadow Robot -> Fingertip Visualizer

This plugin supports presenting the data coming in real time from the Dexterous Hand and from a rosbag.
