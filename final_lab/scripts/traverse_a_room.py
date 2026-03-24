from time import time
from utils.brick import wait_ready_sensors, reset_brick, Motor, EV3UltrasonicSensor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from final_lab.impl.room_traverser import RoomTraverser
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController

MOTOR_POWER = 80

# sensor declaration
left_motor = Motor(Ports.LEFT_MOTOR_PORT)
right_motor = Motor(Ports.RIGHT_MOTOR_PORT)
color_motor = Motor(Ports.COLOR_SENSOR_MOTOR_PORT)
ultrasonic_sensor = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR_PORT)

driving_controller = DrivingController(left_motor, right_motor, MOTOR_POWER)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR, color_motor)
room_traverser = RoomTraverser(driving_controller, color_sensor_controller)

print("Waiting for sensors to be ready...")

wait_ready_sensors(True)

try:
    # here, we appear at the entry of the CORNER SINGLE ROOM, move straight up to 25CM or until detect bed.
    traverse_results = room_traverser.run(4, 25, detect_bed=True)
    bed_found = traverse_results[0] is not None
    bed_color_found = traverse_results[0]
    distance_travelled = traverse_results[1]
    
    if bed_found:
        print(f"Detected bed color: {bed_color_found}, Distance travelled: {distance_travelled:.2f} cm")
        print("DROPPING BLOCK!!!")
        time.sleep(2)
        print("Going back.")
        room_traverser.run(4, -1 * distance_travelled, detect_bed=False)
    else:
        print(f"No bed detected, Distance travelled: {distance_travelled:.2f} cm")
        print("Going back.")
        room_traverser.run(4, -1 * distance_travelled, detect_bed=False)

except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)  
finally:  
    reset_brick()  
    exit()