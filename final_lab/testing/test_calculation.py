import math
dis_wheels_mm = 159
l_rod_mm = 65
l_axle_rod_mm = 69
dia_wheel_mm = 42.6


angle = -29.0
x = 180-abs(angle)
print(f"cos of {x}: {math.cos(math.radians(x))}")
c = math.sqrt(l_rod_mm**2+l_axle_rod_mm**2+2*l_rod_mm*l_axle_rod_mm* \
                            abs(math.cos(math.radians(x))))
print(f"length on unknown side {c}")
phi = math.degrees(math.asin(l_rod_mm*math.sin(math.radians(x))/c))
print(f"turning angle for robot {phi}")
total_ang_for_wheel = phi * dis_wheels_mm / dia_wheel_mm

print(f"turning angle for wheel {total_ang_for_wheel}")
dps = 150
t = total_ang_for_wheel/dps
print(f"turning time for robot {t}")