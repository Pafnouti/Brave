#!/usr/bin/env python
import rospy
import os, subprocess

rospy.loginfo(os.getcwd())

os.chdir("..")

subprocess.run(["node", "start.js"])