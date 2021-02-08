#!/usr/bin/python3.8

import rospy
import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
os.chdir("..")
os.system("node start.js")