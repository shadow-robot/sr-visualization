#!/usr/bin/env python


def func1(all, line, type):
    if all:
        func2(line, type)
    else:
        print("func1", all, line, type)


def func2(line, type):
    print("func2", line, type)


def printargs(a,b,**dict):
    print 'a=%s' % a
    print 'b=%s'
    for k in dict.keys():
        print '%s=%s' % (k,dict[k])

#printargs(x='seven',a='six',b='five',next='four',last='three')

def test_var_kwargs(all, **kwargs):
    print "all?", all
    line = kwargs["line"]
    type = kwargs["type"]
    print "lt", line, type
    # for key in kwargs:
    #     print "another keyword arg: %s: %s" % (key, kwargs[key])

test_var_kwargs(True, line=3, type="control_loops")
#
# func1(True)
# func1(False, 7, "ctrl")


    self.graph_dict_global[type][self.graph_names_global[type][i]].line_to_plot = line_number
    self.graph_dict_global[type][self.graph_names_global[type][i]].plot_all = False
    self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.yaxis.set_tick_params(which='both', labelbottom=True)

    self.graph_dict_global[type][self.graph_names_global[type][i]].ymin = self.global_yaml["graphs"][index]["ranges"][line_number][0]
    self.graph_dict_global[type][self.graph_names_global[type][i]].ymax = self.global_yaml["graphs"][index]["ranges"][line_number][1]
    self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.legend(self.graph_dict_global[type][self.graph_names_global[type][i]].line, [legend_name], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, prop={'size': 7})
    self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = True
    self.graph_dict_global[type][self.graph_names_global[type][i]].update()
    self.graph_dict_global[type][self.graph_names_global[type][i]].draw()


def change_to_all_graphs(self, type):
    if type == "pos_vel_eff":
        index = 0
    elif type == "control_loops":
        index = 1
    ymin, ymax = self.find_max_range(self.global_yaml["graphs"][index])
    i = 0
    # for each graph
        self.graph_dict_global[type][self.graph_names_global[type][i]].ymin = ymin
        self.graph_dict_global[type][self.graph_names_global[type][i]].ymax = ymax
        # for graph in self.graph_names_global[type]:
        self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = False
        self.graph_dict_global[type][self.graph_names_global[type][i]].plot_all = True
        self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.yaxis.set_tick_params(which='both', labelbottom=False)
        self.graph_dict_global[type][self.graph_names_global[type][i]].ax1.legend(self.graph_dict_global[type][self.graph_names_global[type][i]].line, self.global_yaml["graphs"][index]["lines"], bbox_to_anchor=(0.0, 1.0, 1.0, 0.9), framealpha=0.8, loc=3, mode="expand", borderaxespad=0.5, ncol=3, prop={'size': 7})
        self.graph_dict_global[type][self.graph_names_global[type][i]].enabled = True
        self.graph_dict_global[type][self.graph_names_global[type][i]].update()
        self.graph_dict_global[type][self.graph_names_global[type][i]].draw()
        i += 1


