from controllers.color_sensor_controller import ColourSensorController
from data.ports_brick import Ports
from utils.brick import wait_ready_sensors, reset_brick, Motor

color_motor = Motor(Ports.COLOR_MOTOR.value)
color_sensor = Ports.COLOR_SENSOR.value

color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_motor,calibrate=True)

try:
    while True:
        pass
except KeyboardInterrupt:
    reset_brick()

