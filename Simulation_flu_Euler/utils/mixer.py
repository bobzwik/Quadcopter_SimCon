# -*- coding: utf-8 -*-

import numpy as np

def mixer(throttle, pCmd, qCmd, rCmd, quad):
    maxCmd = quad.params["maxCmd"]
    minCmd = quad.params["minCmd"]

    # rCmd = 0

    cmd = np.zeros([4, 1])
    cmd[0] = throttle + pCmd - qCmd + rCmd
    cmd[1] = throttle - pCmd - qCmd - rCmd
    cmd[2] = throttle - pCmd + qCmd + rCmd
    cmd[3] = throttle + pCmd + qCmd - rCmd
    
    cmd[0] = min(max(cmd[0], minCmd), maxCmd)
    cmd[1] = min(max(cmd[1], minCmd), maxCmd)
    cmd[2] = min(max(cmd[2], minCmd), maxCmd)
    cmd[3] = min(max(cmd[3], minCmd), maxCmd)
    
    return cmd

def expoCmd(params, cmd):
    if params["ifexpo"]:
        cmd = np.sqrt(cmd)*10
    
    return cmd

def expoCmdInv(params, cmd):
    if params["ifexpo"]:
        cmd = (cmd/10)**2
    
    return cmd