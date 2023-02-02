#!/usr/bin/env python

import rospy
import sys

def vacuum_cleaning():
    print('start vacuum_cleaning')
    raise NotImplementedError

def inspection():
    print('start inspection')
    raise NotImplementedError


def inspection_advanced():
    print('start inspection_advanced')
    raise NotImplementedError



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)
    if exec_mode == 'cleaning':
        vacuum_cleaning()
    elif exec_mode == 'inspection':
        inspection()
    elif exec_mode == 'inspection_advanced':
        inspection_advanced()
    else:
        print("Code not found")
        raise NotImplementedError
