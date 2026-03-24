dis_wheels_mm = 159
 
l_rod_mm = 65
"""Length of the colour arm rod"""

l_axle_rod_mm = 83.8
"""Distance from wheel axle to the centre of rotation of colour arm"""

dia_wheel_mm = 42.6
"""Diameter of the wheels"""

orienttodeg = dis_wheels_mm/dia_wheel_mm
"""constant for robot turning (e.g. orienttodeg * angle = angle for motors to turn)"""

turn_angle = 45
"""angle to set colour sensor arm for turning"""

dist_axle_intersect_mm = 129.762
"""distance between center of rotation and the intersection when robot about to turn
= 83.8 + 65 sin(45) """

dist_pivot_mm = 79.5
"""distance from intersection when turning"""

dist_move_mm = dist_axle_intersect_mm-dist_pivot_mm
"""distance robot need to move after detecting intersection"""