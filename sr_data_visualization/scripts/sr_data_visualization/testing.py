#!/usr/bin/env python

import yaml
import rospy




class CustomFigCanvas():
    def __init__(self, num_lines, colour = [], ymin = -1, ymax = 1, graph_title="uhoh", ranges=[], legends = [], legend_columns='none', legend_font_size=7, num_ticks=4, xaxis_tick_animation=False, tail_enable=True):
        self.num_lines = num_lines
        print(num_lines)
        print(colour)
        print("ymin:", ymin)
        print("ymax:", ymax)
        print(graph_title)
        print(ranges)


def my_func(data):
    #print(data["graphs"])
    tmp = data["graphs"]
    my_dictionary = {}
    for graphs in data["graphs"]:
        print "------------------------"
        print "type: ", graphs["type"]
        print "number of graphs: ", len(graphs["graph_names"])
        print "lines: "
        for line in graphs["lines"]:
            print"\t", (line)
        print("ranges: ")
        for range in graphs["ranges"]:
            print(range)
        print(graphs["ranges"][0][0])
        ymin = 0
        ymax = 0
        i = 0
        while i < len(graphs["ranges"]) - 1:
            if (graphs["ranges"][i][0] < ymin and graphs["ranges"][i][1] > ymax):
                ymin = graphs["ranges"][i][0]
                ymax = graphs["ranges"][i][1]
            i += 1
        i = 0
        while i < (len(graphs["graph_names"])):
            my_dictionary[graphs["graph_names"][i]] = CustomFigCanvas(len(graphs["lines"]), colour='blue', ymin=ymin, ymax=ymax, ranges=graphs["ranges"], graph_title=graphs["graph_names"][i], legends=graphs["lines"])
            i += 1
        #
        # customFigCanvas_list = []
        # while i < (len(graphs["graph_names"]) - 1):
        #     customFigCanvas_list.append(CustomFigCanvas(len(graphs["lines"]), colour='blue', ymin=graphs["ranges"][i][0], ymax=graphs["ranges"][i][1], graph_title=graphs["graph_names"][i]))
        #     i += 1
       # (self, num_lines, colour
        # =[], ymin = -1, ymax = 1, legends =[], legend_columns='none', legend_font_size=7, num_ticks=4, xaxis_tick_animation=False, tail_enable=True):

       # for graph_name in graphs["graph_names"]:



with open("example.yaml", 'r') as stream:
    try:
        data_loaded = yaml.load(stream)
        my_func(data_loaded)
        hold = data_loaded
    except yaml.YAMLError as exc:
        print(exc)



