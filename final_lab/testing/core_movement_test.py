from controllers.driving_controller import DrivingController

if __name__ == "__main__":
    while True:
        dir = input("T (turn) or S (straight)?")
        x = float(input("x: "))
        dps = float(input("dps: "))
        if dir == "T":
            DC.turn_angle(x,dps)
        elif dir == "S":
            DC.move_straight(x, dps)
        else:
            print("no work")