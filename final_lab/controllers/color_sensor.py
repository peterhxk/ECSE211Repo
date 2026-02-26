from utils.brick import EV3ColorSensor

class ColorSensor (EV3ColorSensor):
    
    def read_colour():
        return
    
    def save_colour_data(file):
        with open(file, mode="w", encoding="utf-8") as outfile:
            while True:
                