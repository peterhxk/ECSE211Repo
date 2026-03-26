from enum import Enum

class Ports(Enum):
    LEFT_MOTOR = "C"
    RIGHT_MOTOR = "B"
    COLOR_SENSOR = 2
    COLOR_MOTOR = "A"
    GRABBER_MOTOR = "D"
    ULTRASONIC_SENSOR = 1
    GYRO_SENSOR = 4

# to import from final_labs/scripts do
# from ..data.ports_brick import Ports

# Access a specific field : `Ports.COLOR_SENSOR.value` for example 
