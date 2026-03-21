from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.bed_sweeping import BedSweeper
from impl.wall_follower import WallFollower

ROOM_TRAVEL_DISTANCE_CM = 30

"""
    Class to handle room traversal. Process is run with the "run" method
    that is called only when the robot has arrived at a room, i.e. at the
    "doorstep". Is meant to detect one of two bed colors: red or green.
"""
class RoomTraverser:
    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor
        self.pilot = pilot
        self.bed_sweeper = BedSweeper(sensor, pilot)
        self.wall_follower = WallFollower(pilot, sensor)

    """
        Mastermind running function to traverse any type of room.
        Will only traverse the room type that is specified through the arguments.
        
        Input:
            - args: dict{
                "corner_room_left_wall_distance": float | None,
                "corner_room_straight_distance": float | None,
                "wall_room_wall_distance": float | None,
                "wall_room_straight_distance": float | None,
                "double_room_wall_distance": float | None,
                "double_room_straight_distance": float | None
            }
    """
    def run(self, args: dict[str, float]):
        pass