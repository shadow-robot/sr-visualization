#!/usr/bin/env python

import rospy
from control_msgs.msg import JointControllerState



def func(name):
    def func2(value):
        x = name
        print(x)
        print(value.set_point, 0)
        print(value.process_value, 1)
        print(value.process_value_dot, 2)
        print(value.error, 3)
        print(value.command, 4)
    return func2


control_loop_callback_dict = {}

rospy.init_node("dynamic_testing_3")

tmp_func = func("name1")

sub = rospy.Subscriber('/sh_rh_ffj0_position_controller/state', JointControllerState, callback=tmp_func,  queue_size=1)
control_loop_callback_dict["name1"] = tmp_func

tmp_func = func("name2")

sub = rospy.Subscriber('/sh_rh_ffj3_position_controller/state', JointControllerState, callback=tmp_func,  queue_size=1)
control_loop_callback_dict["name2"] = tmp_func

rospy.spin()
