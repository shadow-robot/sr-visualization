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


rospy.init_node("dynamic_testing_3")

sub = rospy.Subscriber('/sh_rh_ffj0_position_controller/state', JointControllerState, callback=func(("graph_name")),  queue_size=1)

sub = rospy.Subscriber('/sh_rh_ffj3_position_controller/state', JointControllerState, callback=func(("graph_name2")),  queue_size=1)

rospy.spin()
