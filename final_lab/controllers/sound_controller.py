from utils.sound import Sound
from concurrent import futures
import threading
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
        self._stop_event = threading.Event()
        self._active_sequence = []

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

        self._get_executor().submit(_play)

    def play_mission_complete(self):
        """Super Mario Bros. 1-UP jingle.

        E5 G5 E6 C6 D6 G6
        Played asynchronously; returns immediately.
        """
        sequence = [
            Sound(duration=1, pitch="E4", volume=90),
            Sound(duration=1, pitch="G5", volume=90),
            Sound(duration=1, pitch="E5", volume=90),
            Sound(duration=1, pitch="C6", volume=90),
            Sound(duration=1, pitch="D6", volume=90),
            Sound(duration=1, pitch="G6", volume=90),
        ]

        self._stop_event.clear()
        self._active_sequence = sequence

        def _play():
            self.note_playing = True
            for note in sequence:
                if self._stop_event.is_set():
                    break
                note.play()
                self._stop_event.wait(note._duration - 0.05)
            self.note_playing = False

        self._get_executor().submit(_play)

    def play_success(self):
        """Short upbeat chime: quick two-note rise.

        Sequence: E5 - G5 (short)
        Played asynchronously; returns immediately.
        """
        sequence = [
            Sound(duration=1, pitch="E5", volume=80),
            Sound(duration=1, pitch="G5", volume=80),
        ]

        self._stop_event.clear()
        self._active_sequence = sequence

        def _play():
            self.note_playing = True
            for note in sequence:
                if self._stop_event.is_set():
                    break
                note.play()
                self._stop_event.wait(note._duration - 0.05)
            self.note_playing = False

        self._get_executor().submit(_play)

    def stop_all(self):
        """Stop any currently playing note."""
        self._stop_event.set()
        for s in self._active_sequence:
            s.stop()
        self._active_sequence = []
        for s in self.sounds:
            s.stop()
        self._executor.shutdown(wait=False, cancel_futures=True)
        self.note_playing = False
        self.play_requested = False
        self._stopping = False

    def wait_done(self, polling_interval=0.05):
        """Blocking function that waits until notes are done playing."""

        time.sleep(polling_interval)
        while self.note_playing:
            time.sleep(polling_interval)
