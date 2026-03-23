#!/usr/bin/env python3
import math
import statistics
import threading
from enum import Enum
from collections import deque
import sys
from tqdm import tqdm
sys.path.append("../utils")
from brick import EV3GyroSensor, Motor,reset_brick
import time

grabber_motor = Motor("D")

try :
    grabber_motor.set_dps(50)
    starter = time.time()
    while True:
        if time.time() - starter > 1:
            starter = time.time()
            print(grabber_motor.get_dps())

except KeyboardInterrupt:
    reset_brick()