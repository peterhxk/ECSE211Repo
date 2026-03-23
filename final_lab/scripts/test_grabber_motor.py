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

grabber_motor = EV3GyroSensor("D")

try :
    grabber_motor.set_dps(50)
    while True:
        pass

except KeyboardInterrupt:
    reset_brick()