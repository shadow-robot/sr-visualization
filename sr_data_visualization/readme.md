# sr_data_visualization

This is a package to graphically display data coming out of the Dexterous Hand. 

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

## Configuration 

Inside the `config` folder you will find a file called `data_visualiser_parameters.yaml`. From this file you can change parameters such as:

* Data ranges
* Units displayed on graph legends
* Remap topics
* Font sizes
* Line colours

## Testing 

This package includes a script to fetch a bag file for testing purposes. This allows the gui to be used without having a physical or simulated hand running or connected. 

To fetch this bag file, run:
``` 
rosrun sr_data_visualization download_testing_bag.py 
```
Once this program has finished, launch the gui (and the bag file) with:
```
roslaunch sr_data_visualization data_visualizer_testing.launch
```


