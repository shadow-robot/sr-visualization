
import yaml
import numpy as np
import os
import signal
import rospy
import rospkg
import string

from control_msgs.msg import JointControllerState


def control_loop_cb_dict(data):
    tmp = "s"


# output_elements = ['id1', 'id2']
#
# def create_callback(output):
#      def callback(input_value):
#         if output == 'id1':
#             # do something
#         elif output == 'id2':
#             # do something different
#     return callback
#
# for output_element in output_elements:
#      dynamically_generated_function = create_callback(output_element)
#     app.callback(Output(output_element, '...'), [Input(...)])(dynamically_generated_function)

def make_func(value_to_print):
    def _function():
        print value_to_print
    return _function

#You can generate a list of these and store, again at runtime.

my_functions = [make_func(i) for i in range(1, 11)]
for each in my_functions:
    each()

def create_a_function(*args, **kwargs):

    def function_template(*args, **kwargs):
        pass

    return function_template

my_new_function = [] #= create_a_function()
my_new_function.append(create_a_function())
my_new_function.append(create_a_function())

def my_func(data):
    # print(data["graphs"])
    tmp = data["graphs"]
    graph_dict = {}
    subs = []
    global_yaml = data
    control_loop_cb_dict = {}
    for graphs in data["graphs"]:
        if graphs["type"] == 'control_loops':
            i = 0
            while i < len(graphs["graph_names"]):
                sub_namespace = graphs["topic_namespace_start"] + graphs["graph_names"][i] + graphs["topic_namespace_end"]
                subs.append(rospy.Subscriber(sub_namespace, JointControllerState, control_loop_cb_dict[graphs["graph_names"][i]], queue_size=1))
                print(sub_namespace)
                i += 1

with open("example.yaml", 'r') as stream:
    try:
        data_loaded = yaml.load(stream)
        my_func(data_loaded)
    except yaml.YAMLError as exc:
        print(exc)
    done = "s"

