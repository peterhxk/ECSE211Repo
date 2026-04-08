from utils.sound import Sound

note = "A4"   # note name (e.g. "C4", "G5") or frequency in Hz
volume = 50   # 0–100
duration = 1  # seconds

s = Sound(pitch=note, volume=volume, duration=duration)
s.play()
s.wait_done()
