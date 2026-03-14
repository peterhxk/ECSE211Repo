import math
dis_wheels_mm = 159
l_rod_mm = 65
l_axle_rod_mm = 69
dia_wheel_mm = 42.6


angle = 9.5
x = 180-abs(angle)
print(f"cos of {x}: {math.cos(math.radians(x))}")
c = math.sqrt(l_rod_mm**2+l_axle_rod_mm**2+2*l_rod_mm*l_axle_rod_mm* \
                            abs(math.cos(math.radians(x))))
print(f"length on unknown side {c}")
phi = math.degrees(math.asin(l_rod_mm*math.sin(math.radians(x))/c))
print(f"turning angle for robot {phi}")