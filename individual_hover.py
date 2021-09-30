#!/usr/bin/env python

from __future__ import print_function

from pycrazyswarm import *
import numpy as np
import time
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for cf in allcfs.crazyflies:
        print(cf.id)
        cf.takeoff(0.5, 2.5)

    print("press button to continue")
    #swarm.input.waitUntilButtonPressed()
    time.sleep(5)

    for cf in allcfs.crazyflies:
        cf.land(0.04, 2.5)


if __name__ == "__main__":
    main()
