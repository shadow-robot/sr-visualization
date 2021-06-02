#!/usr/bin/env python3
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
#

from __future__ import absolute_import
import rospy
import os
import yaml
import rospkg

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class PidLoader(object):

    """
    Loads pid parameters of each controller in parameters_dict from a yaml file
    """

    def __init__(self):
        pass

    def get_settings(self, param_name):
        param_dict = {}
        if len(param_name) == 2:
            try:
                tmp_dict = rospy.get_param(param_name[0])
            except KeyError:
                return -1
            for item in list(tmp_dict.items()):
                param_dict["pos/" + item[0]] = item[1]

            try:
                tmp_dict = rospy.get_param(param_name[1])
            except KeyError:
                return -1
            for item in list(tmp_dict.items()):
                param_dict["vel/" + item[0]] = item[1]
        else:
            try:
                param_dict = rospy.get_param(param_name)
            except KeyError:
                return -1
        return param_dict


class PidSaver(object):

    """
    Saves pid parameters of each controller in parameters_dict in a yaml file
    """

    def __init__(self, file_path):
        self.path = file_path

    def save_settings(self, param_path, parameters_dict):
        f = open(self.path, 'r')
        document = ""
        for line in f.readlines():
            document += line
        f.close()

        yaml_config = yaml.load(document)

        for item in list(parameters_dict.items()):
            if "pos/" in item[0]:
                yaml_config[param_path[0]]["position_pid"][
                    item[0].split("pos/")[1]] = item[1]
            elif "vel/" in item[0]:
                yaml_config[param_path[0]]["velocity_pid"][
                    item[0].split("vel/")[1]] = item[1]
            else:
                yaml_config[param_path[0]][param_path[1]][item[0]] = item[1]

        full_config_to_write = yaml.dump(yaml_config, default_flow_style=False)

        f = open(self.path, 'w')
        f.write(full_config_to_write)
        f.close()


if __name__ == '__main__':
    path_to_config = "~"
    try:
        path_to_config = os.path.join(
            rospkg.RosPack().get_path("sr_edc_controller_configuration"))

        pid_saver = PidSaver(
            path_to_config + "/sr_edc_mixed_position_velocity_joint_controllers.yaml")
        pid_saver.save_settings(
            ["sh_wrj2_mixed_position_velocity_controller", "pid"], {"d": 1.0})
    except Exception:
        rospy.logwarn(
            "couldnt find the sr_edc_controller_configuration package")
