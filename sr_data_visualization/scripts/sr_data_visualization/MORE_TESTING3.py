#!/usr/bin/env python

import rospy
from control_msgs.msg import JointControllerState
from diagnostic_msgs.msg import DiagnosticArray
import sys
import time
import yaml

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


mech_list = []
mech_list.append(3)
mech_list.append(6)
mech_list.append(7)
mech_list.append(8)
mech_list.append(11)
mech_list.append(12)
mech_list.append(13)
mech_list.append(16)
mech_list.append(17)
mech_list.append(18)
mech_list.append(21)
mech_list.append(22)
mech_list.append(23)
mech_list.append(24)
mech_list.append(25)
mech_list.append(26)
mech_list.append(27)
mech_list.append(28)
mech_list.append(29)
mech_list.append(30)
mech_vals = []
mech_vals.append(4)
mech_vals.append(5)
mech_vals.append(7)
mech_vals.append(8)
mech_vals.append(9)
mech_vals.append(10)
mech_vals.append(11)
mech_vals.append(12)
mech_vals.append(13)
mech_vals.append(28)
mech_vals.append(29)
first = True
def stat_cb(value):
    global first
    global data_loaded
    tmp = data_loaded
    if first:
        if len(value.status) > 1:
            for item in mech_list:
                print value.status[item].name, item
            print()
        first = False
    else:
        # for each graph
        if len(value.status) > 1:
            for key, name in data_loaded:
                tmp = name
                # for each line
                print(value.status[name].name, value.status[value].values[4].value)
                print(value.status[name].values[5].value)
                print(value.status[name].values[7].value)


def diagnostic_cb(value):
# Only add data to graphs once they've all been created
    j = 0
    global motor_stat_keys
    global motor_stat_vals
    # for each graph
    if len(value.status) > 1:
        for key, name in motor_stat_keys.iteritems():
            # for each line
            print(value.status[name].name, value.status[name].values[4].key, value.status[name].values[4].value)
            # print(value.status[name].values[5].value)
            # print(value.status[name].values[7].value)
            j += 1
        for val in mech_vals:
            print value.status[3].values[val].key, val



# self.motor_stat_graph_2.addData(float(value.status[6].values[4].value) * (0.066666667), 0)
# self.motor_stat_graph_2.addData(float(value.status[6].values[5].value) * (0.066666667), 1)
# self.motor_stat_graph_2.addData(float(value.status[6].values[7].value) * (0.1), 2)
# self.motor_stat_graph_2.addData(float(value.status[6].values[8].value) * 300, 3)
# self.motor_stat_graph_2.addData(value.status[6].values[9].value, 4)
# self.motor_stat_graph_2.addData(float(value.status[6].values[10].value) * (0.01), 5)
# self.motor_stat_graph_2.addData(value.status[6].values[11].value, 6)
# self.motor_stat_graph_2.addData(value.status[6].values[12].value, 7)
# self.motor_stat_graph_2.addData(float(value.status[6].values[13].value) * (0.02), 8)
# self.motor_stat_graph_2.addData(float(value.status[6].values[28].value) * (0.05), 9)
# self.motor_stat_graph_2.addData(float(value.status[6].values[29].value) * (5), 10)

#motor_stat_graph_1.addData(float(value.status[3].values[4].value) * (0.066666667), 0)  # sg left #done
def my_func(data):
    global motor_stat_keys
    motor_stat_keys = data

def funcs(data):
    global motor_stat_vals
    motor_stat_vals = data

with open("mech_stat_val_keys.yaml", 'r') as stream:
    try:
        data_loaded = yaml.load(stream)
        my_func(data_loaded)
    except yaml.YAMLError as exc:
        print(exc)



sub = rospy.Subscriber('/diagnostics', DiagnosticArray, diagnostic_cb, queue_size=1)


rospy.spin()
