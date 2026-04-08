from utils.sound import Sound

# ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
note = "A2"   # note name (e.g. "C4", "G5") or frequency in Hz
volume = 100   # 0–100
duration = 1  # seconds

s = Sound(pitch=note, volume=volume, duration=duration)
s.play()
s.wait_done()
