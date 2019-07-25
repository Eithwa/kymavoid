#!/usr/bin/env python3
# -*- coding: utf-8 -*-+


""" ros """
import rospy
import rospkg

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

""" tool """
import math
import numpy as np

SCAN_LIMIT = 999
SCAN_NUM = 120

class NodeHandle(object):
    def __init__(self):
        
        """ pub """
        self.pub_black = rospy.Publisher('vision/BlackRealDis',Int32MultiArray, queue_size = 1)
        self.pub_red = rospy.Publisher('vision/redRealDis',Int32MultiArray, queue_size = 1)

        """ sub """
        rospy.Subscriber("ob_scan",LaserScan,self.Sub_Black) # red and black scan
        rospy.Subscriber("scan",LaserScan,self.Sub_Red)  # red scan
    
        print('init_node')
        
    def Sub_Black(self,msg):
        if(msg is not None):
            scan = [999]*SCAN_NUM
            for i in range(len(msg.ranges)):
                if(msg.ranges[i] == math.inf):
                    scan[i] = SCAN_LIMIT
                elif(msg.ranges[i] == math.isnan):
                    scan[i] = 20
                else:
                    scan[i] = int(round(msg.ranges[i],2)*100)
            pub_msg = Int32MultiArray()
            pub_msg.data = scan
            self.pub_black.publish(pub_msg)

    def Sub_Red(self,msg):
        if(msg is not None):
            scan = [999]*SCAN_NUM
            for i in range(len(msg.ranges)):
                if(msg.ranges[i] == math.inf):
                    scan[i] = SCAN_LIMIT
                elif(msg.ranges[i] == math.isnan):
                    scan[i] = 20
                else:
                    scan[i] = int(round(msg.ranges[i],2)*100)

            pub_msg = Int32MultiArray()
            pub_msg.data = scan
            self.pub_red.publish(pub_msg)

if __name__ == '__main__':
    main()
