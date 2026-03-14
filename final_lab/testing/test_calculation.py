import math
dis_wheels_mm = 159
l_rod_mm = 65
l_axle_rod_mm = 69
dia_wheel_mm = 42.6


angle = 56.333333333333336
x = 180-abs(angle)
c = math.sqrt(l_rod_mm**2+l_axle_rod_mm**2+2*l_rod_mm*l_axle_rod_mm* \
                            math.cos(x))
print(f"length on unknown side {c}")
phi = math.degrees(math.asin(l_rod_mm*math.sin(x)/c))
print(f"turning angle for robot {phi}")