#!/usr/bin/env python

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
    url = "https://www.dropbox.com/s/4iavfgfcbtpfmoh/biotacs_and_everything_else.bag?dl=1"
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

