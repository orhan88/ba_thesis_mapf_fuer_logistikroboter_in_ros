#! /usr/bin/env python3
'''
    This function starts the GUI-application for the simulation of Multi-Agent Path Finding 
'''
from mapf_simulation.control import VizControl
import sys

def main():
    '''
    This function starts the GUI-application for the simulation of Multi-Agent Path Finding 
    '''
    control = VizControl()
    control.run()

if __name__ == '__main__':
    sys.exit(main())

