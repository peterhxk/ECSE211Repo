dis_wheels_mm = 159
l_rod_mm = 65
l_axle_rod_mm = 69
dia_wheel_mm = 42.6
# constant for robot turning (e.g. orienttodeg * angle = angle for motors to turn)
orienttodeg = dis_wheels_mm/dia_wheel_mm
# angle for arm that aligns with pivot
angle_pivot = 81.247 # cos-1(10.5/69)