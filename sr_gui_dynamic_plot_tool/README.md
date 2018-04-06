# Sr-gui-dynamic-plot-tool

Package that offers rqt GUI plugin to easily generate useful plots for Agile Grasper and hand E.

# Usage

The plugin can be started from rqt. In the top navigation bar select plugins, then Shadow Robot and finally
Dynamic Plot Tool.

On plugin start up this bar will be shown:

![alt text](https://github.com/shadow-robot/sr-visualization/blob/F%23SRC-1485_implement_rqt_dyn_plots/sr_gui_dynamic_plot_tool/BarPlugin.png)

From this it's possible to select the script with the topics and plot configurations to run.<br/>

If you want to plot **Hand E data**, select the **hand_e_default_configuration.py** script.<br/>
If you want to plot **Agile Grasper data**, select the **agile_grasper_default_configuration.py** script.<br/>
Once you selected the right script click **Run** button which will generate the plot selection interface
as shown in the images below.<br/>


# Example view Agile Grasper

![alt text](https://github.com/shadow-robot/sr-visualization/blob/F%23SRC-1485_implement_rqt_dyn_plots/sr_gui_dynamic_plot_tool/AgileGrasper_example.png)


# Example view Hand E

![alt text](https://github.com/shadow-robot/sr-visualization/blob/F%23SRC-1485_implement_rqt_dyn_plots/sr_gui_dynamic_plot_tool/HandE_example.png)

In those GUI select the Hand, the Finger, the Joint and the Configuration you are interested in plotting.<br/>
By clicking **Plot** a new instance of rqt_multiplot with the requested plot will be generated.<br/>
If you want to plot another joint, just repeat the process of selecting the hand, finger, joint and configuration and click again on Plot to open the requested plot in a separate window.
