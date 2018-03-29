# Sr-gui-dynamic-plot-tool

Package that offers rqt GUI plugin to easily generate useful plots for agile grasper and hand E.

# Usage

The plugin can be started from rqt. In the top navigation bar select plugins, then Shadow Robot and finally
Dynamic Plot Tool.

If you want to plot hand E data, select the hand_e_default_configuration.py script, while if you want to plot
agile grasper data select the agile_grasper_default_configuration.py script. 

# Example view Agile Grasper




# Example view Hand E


In those GUI select the hand, the finger, the joint and the data you are interested in plotting.
Clicking 'PLOT' will open a new instance of rqt_multiplot with the requested plot.
If you want another plot, just repeat the process by selecting the hand, finger, joint and configuration you want and by clicking PLOT the requested plot will be opened in a separate window.