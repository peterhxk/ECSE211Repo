from utils.brick import EV3ColorSensor
import time
import json

class ColorSensor (EV3ColorSensor):

    def __init__(self, port, mode="component", bp=None):
        super(ColorSensor, self).__init__(port, bp)
        self.set_mode(mode)
    
    def read_colour():
        cur_rgb = super.get_rgb()
        cur_r = cur_rgb[0]
        cur_g = cur_rgb[1]
        cur_b = cur_rgb[2]
        with open("data/colour.json", "r") as f:
            colours = json.load(f)
        colour_to_distance = {}
        for colour in colours:
            r = colour[0]
            g = colour[1]
            b = colour[2]

            d = ((cur_r - r)^2+(cur_g - g)^2+(cur_b - b)^2)^(0.5)
            colour_to_distance[colour] = d
        
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

