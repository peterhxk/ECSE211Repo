from utils.sound import Sound
from concurrent import futures
import time

NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
TEMPO = 0.5


class SoundController(Sound):
    def __init__(self, duration=0.3, volume=80):
        super().__init__(duration=duration, volume=volume)
        self.sounds = [Sound(duration=duration, pitch=note, volume=volume) for note in NOTES]
        self.note_playing = False
        self.play_requested = False
        self._executor = futures.ThreadPoolExecutor(max_workers=1)

    def request_play(self):
        """Signal that a note should be played on the next update."""
        self.play_requested = True

    def play_note(self, note_index):
        """Play the note at note_index asynchronously.

        Does nothing if a note is already playing or the index is out of range.
        """
        if self.note_playing or not self.play_requested:
            return
        if note_index < 0 or note_index >= len(self.sounds):
            self.play_requested = False
            return

        note = self.sounds[note_index]

        def _play():
            self.note_playing = True
            note.play()
            note.wait_done()
            self.note_playing = False
            self.play_requested = False

        self._executor.submit(_play)

    def play_mission_complete(self):
        """Super Mario Bros. 1-UP jingle.

        E5 G5 E6 C6 D6 G6
        Played asynchronously; returns immediately.
        """
        # Super Mario Bros. 1-UP jingle
        sequence = [
            Sound(duration=0.12, pitch="E5", volume=80),
            Sound(duration=0.12, pitch="G5", volume=80),
            Sound(duration=0.12, pitch="E6", volume=80),
            Sound(duration=0.12, pitch="C6", volume=80),
            Sound(duration=0.12, pitch="D6", volume=80),
            Sound(duration=0.25, pitch="G6", volume=80),
        ]

        def _play():
            self.note_playing = True
            for note in sequence:
                note.play()
                time.sleep(note._duration)
            self.note_playing = False

        self._executor.submit(_play)

    def play_success(self):
        """Short upbeat chime: quick two-note rise.

        Sequence: E5 - G5 (short)
        Played asynchronously; returns immediately.
        """
        sequence = [
            Sound(duration=1, pitch="E5", volume=80),
            Sound(duration=1, pitch="G5", volume=80),
        ]

        def _play():
            self.note_playing = True
            for note in sequence:
                note.play()
                time.sleep(note._duration)
            self.note_playing = False

        self._executor.submit(_play)

    def stop_all(self):
        """Stop any currently playing note."""
        for s in self.sounds:
            s.stop()
        self.note_playing = False
        self.play_requested = False
