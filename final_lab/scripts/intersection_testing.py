import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor_motor = Motor("A")

driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(2, color_sensor_motor)

line_follower = SweepLineFollower(color_sensor_controller, driving_controller)
wait_ready_sensors(True)

try:
    line_follower.turn_logic("left")
    '''
    driving_controller.move_distance(1000)

    time.sleep(1)
    while True:
        print(color_sensor_controller.get_rgb())
        time.sleep(0.1)

'''
    '''
    driving_controller.pivot_90("left")
    print("pivot 90 left")
    time.sleep(4)

    driving_controller.pivot_90("right")
    print("pivot 90 right")
    time.sleep(6)

    driving_controller.move_distance(100)
    print("move 100 mm")
    time.sleep(4)

    driving_controller.pivot_90("left")
    print("pivot 90 left")
    time.sleep(4)

    driving_controller.turn_in_place(360)
    print("turn in place, clockwise?")
    time.sleep(10)

    driving_controller.turn_in_place(-360)
    print("turn in place, counter-clockwise?")
    time.sleep(10)
'''

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
