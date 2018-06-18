#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
import os

def cb(srv):
    r = SetBoolResponse()
    next_state = 0
    if srv.data:
        next_state = 9
    msg = msg.format(next_state)
    expected_result = expected_result.format(echo_to_bit[next_state])
    if ring_light_state is None or ring_light_state != next_state:
        for i in range(number_of_retries):
            read = os.popen(msg).read()
            r.message = '{}; after {} tries'.format(read, i)
            if expected_result == read:
                r.success = True
                ring_light_state = next_state
                break
        else:
            r.success = False
    else:
        r.message = 'ring light is already in the requested state.'
        r.success = True
    return r

echo_to_bit = {
    0: 0,
    1: 29,
    2: 58
    3: 87,
    4: 116,
    5: 145,
    6: 174,
    7: 203,
    8: 232,
    9: 255
    }
msg = 'echo "{}" | nc 192.168.102.114 3000'
expected_result = 'Setting light to: {}\n'
ring_light_state = None
number_of_retries = 5

rospy.init_node('ring_light_switch')
s = rospy.Service('~setbool', SetBool, cb)
rospy.spin()