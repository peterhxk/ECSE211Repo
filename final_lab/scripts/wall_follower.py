from utils.brick import wait_ready_sensors, reset_brick, Motor, EV3UltrasonicSensor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.wall_follower import WallFollower

# sensor declaration
left_motor = Motor("D")
right_motor = Motor("C")
ultrasonic_sensor = EV3UltrasonicSensor(1)


driving_controller = DrivingController(left_motor, right_motor)
print("Waiting for sensors to be ready...")
wait_ready_sensors(True)

try:
    follower = WallFollower(driving_controller, ultrasonic_sensor)
    follower.run(10, 30)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)
finally:    
    exit()