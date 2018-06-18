#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
import os

def cb(srv):
    msg = 'echo "{}" | nc 192.168.102.114 3000'
    r = SetBoolResponse()
    if srv.data:
        msg = msg.format(9)
    else:
        msg = msg.format(0)
    for i in range(5):
        read = os.popen(msg).read()
        r.message = read
        if 'Setting light to:' in read:
            r.success = True
            break
    else:
        r.success = False
    return r

rospy.init_node('ring_light_switch')
s = rospy.Service('~setbool', SetBool, cb)
rospy.spin()