from utils.brick import EV3ColorSensor
import time
import json

def normalize_colours(r, g, b):
    total = r+g+b
    if total == 0:
        return 0,0,0
    return r/total, g/total, b/total

class ColorSensor (EV3ColorSensor):

    def __init__(self, port, mode="component", bp=None):
        super().__init__(port)
    
    def read_colour(self):
        cur_rgb = self.get_rgb()
        cur_r = cur_rgb[0]
        cur_g = cur_rgb[1]
        cur_b = cur_rgb[2]
        # nor_r, nor_g, nor_b = normalize_colours(cur_r,cur_g,cur_b)
        # print((nor_r, nor_g, nor_b))
        print(cur_rgb)
        with open("data/colours.json", "r") as f:
            colours = json.load(f)
        colour_to_distance = {}
        for colour, rgb in colours.items():
            r = int(rgb[0])
            g = int(rgb[1])
            b = int(rgb[2])

            distance = ((cur_r - r)**2+(cur_g - g)**2+(cur_b - b)**2)**(1/2)
            colour_to_distance[colour] = distance
        
        shortest_d = None
        closest_colour = None
        for colour, d in colour_to_distance.items():
            if not closest_colour or d < shortest_d:
                shortest_d = d
                closest_colour = colour
        return closest_colour
    
    def save_colour_data(file):
        with open(file, mode="w", encoding="utf-8") as outfile:
            count = 0
            while count < 50:
                outfile.write(f"{ColorSensor.get_rgb()}\n")
                time.sleep(0.1)

