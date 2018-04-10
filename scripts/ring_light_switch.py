#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
import os

def cb(srv):
    if srv.data:
        os.system('echo "9" | nc 192.168.102.114 3000')
        os.system('echo "9" | nc 192.168.102.114 3000')
        os.system('echo "9" | nc 192.168.102.114 3000')
    else:
        os.system('echo "0" | nc 192.168.102.114 3000')
        os.system('echo "0" | nc 192.168.102.114 3000')
        os.system('echo "0" | nc 192.168.102.114 3000')
    r = SetBoolResponse()
    r.success = True
    r.message = 'muh'
    return r

rospy.init_node('ring_light_switch')
s = rospy.Service('~setbool', SetBool, cb)
rospy.spin()