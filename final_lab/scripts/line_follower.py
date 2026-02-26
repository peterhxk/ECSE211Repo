from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColorSensor
from controllers.driving_controller import DrivingController


#Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor = ColorSensor(2)

driving_controller = DrivingController(left_motor, right_motor)

wait_ready_sensors(True)
driving_controller.drive(color_sensor)

try:
    # colour = color_sensor.read_colour()
    # print(colour)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()