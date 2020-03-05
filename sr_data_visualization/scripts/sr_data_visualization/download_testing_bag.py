#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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

import os
import sys
import rospy
import rospkg
import threading
import urllib


def show_progress(event):
    while not event.is_set():
        sys.stdout.write('.')
        sys.stdout.flush()
        event.wait(1)
    print "\n"

bag_file = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'), 'biotacs_and_everything_else.bag')

if not os.path.isfile(bag_file):
    sys.stdout.write("Bag file for testing not found, downloading now. This may take a few minutes .")
    sys.stdout.flush()
    url = "https://www.dropbox.com/s/6vfxvmla4hcsbzd/biotacs_and_everything_else.bag?dl=1"
    file_path = os.path.join(rospkg.RosPack().get_path('sr_data_visualization'))
    trigger = threading.Event()
    t = threading.Thread(target=show_progress, args=(trigger,))
    t.start()
    urllib.urlretrieve(url, bag_file)
    trigger.set()
    print "\n"
    print "File downloaded: ", bag_file
    sys.exit()
else:
    print "Bag file already exists, doing nothing."
    sys.exit()
