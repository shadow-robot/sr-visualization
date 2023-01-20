## SR_HEALTH_CHECK PACKAGE

Package that contains code of the Health Check GUI

The purpose of the GUI is to allow the user running health checks of a Shadow Hand in order to get its health status. The information can be used to understand the current state of the hand any if there are any issues which might require support from the Shadow team.
The Run tab allows to select specific or all tests to be executed and stop the execution. The View tab presents the data collected for the health checks at different points of time. In case of a failed check, its font will be red coloured and expanding the tree with highlighted problems. In case of any performance issues run the checks and contact the Shadow team for support.

### How to use it

The GUI can be launched as an rqt plugin:

```
rqt
```

and go to Plugins -> Shadow Robot -> Health Check
