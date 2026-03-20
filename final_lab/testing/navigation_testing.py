import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor_motor = Motor("A")

pilot = DrivingController(left_motor, right_motor, 150)
sensor = ColourSensorController(2, color_sensor_motor)

follow = SweepLineFollower(sensor, pilot)
wait_ready_sensors(True)

try:
    while True:
        ans1 = input("1. Color sensor 2. Driving 3. Sweeper or 4. Exit (e)? ")

        if ans1 in ["1", "c", "C"]:
            while True:
                ans2 = input("Color sensor:\n 1. Read color 2. Select arm angle or 3. Exit (e)? ")
                if ans2 == "1":
                    print("Reading color sensor...")
                    print(f"Read {sensor.read_colour()}")
                elif ans2 == "2":
                    angle = float(input("Set angle: "))
                    sensor.set_arm_at_angle(angle)
                elif ans2 in ["3", "e", "E"]:
                    break
                else:
                    print("Wrong input\n")

        elif ans1 in ["2", "d", "D"]:
            while True:
                ans2 = input("Driving controller:\n 1. Move forward 2. Pivot or 3. Exit (e)? ")
                if ans2 == "1":
                    dist = float(input("Distance (mm): "))
                    pilot.move_distance(dist)
                elif ans2 == "2":
                    dir = input("Direction (left, right): ")
                    pilot.pivot_90(dir)
                elif ans2 in ["3", "e", "E"]:
                    break
                else:
                    print("Wrong input\n")

        elif ans1 in ["3", "s", "S"]:
            while True:
                ans2 = input("Sweeper controller:\n 1. Move forward 2. Turn Logic or 3. Exit (e)? ")
                if ans2 == "1":
                    max_dist = float(input("Distance (mm): "))
                    force = int(input("Force? True = 1, False = 0 : "))
                    if force == 0:
                        follow.move_forward(max_dist, False)
                    else:
                        follow.move_forward(max_dist, True)
                elif ans2 == "2":
                    dir = input("Direction (left, right): ")
                    follow.turn_logic(dir)
                elif ans2 in ["3", "e", "E"]:
                    break
                else:
                    print("Wrong input\n")

        elif ans1 in ["4", "e", "E"]:
            break

        else:
            print("Wrong input\n")


except KeyboardInterrupt:

    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
