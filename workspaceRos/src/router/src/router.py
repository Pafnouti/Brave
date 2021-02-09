#!/usr/bin/env python

import rospy

from routeur import *

from controller.msg import Line
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Int32





class Router():
    def __init__(self, rosrate=0.1):
        self.routeur = Routeur()

        self.rate = rospy.Rate(rosrate)

        self.activated = False
        self.got_target = False
        self.target = (0, 0)
        self.state = (0, 0)

        self.pub_line = rospy.Publisher('/Line', Line, queue_size=32)

        rospy.Subscriber('/Routing', Bool, self._callback_routing)
        rospy.Subscriber('/Target', Pose2D, self._callback_target)
        rospy.Subscriber('/State', Pose2D, self._callback_state)


    def _callback_target(self, msg):
        self.target = (msg.x, msg.y)
        self.got_target = True
        

    def _callback_state(self, msg):
        self.state = np.array([msg.x, msg.y])

    def _callback_routing(self, msg):
        self.activated = msg.data
        self.main()
    
    def main(self):
        if self.activated and self.got_target:
            print('yes')
            traj, iso = self.run(self.state, self.target)
            self.line = Line()
            self.line.xa = self.state.[0]
            self.line.ya = self.state.[1]
            self.line.xb = traj[0]
            self.line.yb = self.target[1]
            self.pub_line.publish(self.line)
        else:
            print('no')

    


        
    



if __name__ == "__main__":
    rospy.init_node('router', anonymous=True)
    routeur = Routeur(rosrate=0.01)

    while not rospy.is_shutdown():
        routeur.main()
        routeur.rate.sleep()
