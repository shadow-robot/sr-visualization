## SR_HEALTH_CHECK PACKAGE

Package that contains the code of the Health Check GUI

The purpose of the GUI is to allow the user to run health checks on a Shadow Hand in order to get its health status. The information can be used to understand the hand's current state and if any issues might require support from the Shadow team.
The `Run` tab allows the user to select which tests are to be executed and can also stop their execution. The `View` tab reports the data collected for the health checks at different points in time. In case of a failed check, its font will be red coloured and expanding the tree will allow the user to understand the cause of each problem. 

Checks failing are not an unequivocal sign of the hand malfunctioning, but in case of any performance issues, run the checks and contact the Shadow team for support.

Every check class has a PASSED_THRESHOLDS class variable defining the passing threshold for the checks, with the exact conditions being listed in the `has_passed()` or `has_single_passed()` method of the check.

### How to use it

The GUI can be launched as an rqt plugin:

```
rqt
```

and go to Plugins -> Shadow Robot -> Health Check
