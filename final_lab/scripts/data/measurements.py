dis_wheels_mm = 159
 
l_rod_mm = 65
"""Length of the colour arm rod"""

l_axle_rod_mm = 83.8
"""Distance from wheel axle to the centre of rotation of colour arm"""

dia_wheel_mm = 42.6
"""Diameter of the wheels"""

# constant for robot turning (e.g. orienttodeg * angle = angle for motors to turn)
orienttodeg = dis_wheels_mm/dia_wheel_mm
# angle for arm that aligns with pivot
angle_pivot = 81.247 # cos-1(10.5/69)