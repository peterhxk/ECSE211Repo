from utils.brick import wait_ready_sensors, reset_brick, Motor, EV3ColorSensor

#Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor = EV3ColorSensor(2)

wait_ready_sensors(True)



reset_brick()