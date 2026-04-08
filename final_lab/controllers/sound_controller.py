from utils.sound import Sound
from concurrent import futures
import threading
import time
from typing import Optional

import simpleaudio as sa

NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
TEMPO = 0.5

_POLL_S = 0.02


class SoundController(Sound):
    def __init__(self, duration=0.3, volume=80):
        super().__init__(duration=duration, volume=volume)
        self.sounds = [Sound(duration=duration, pitch=note, volume=volume) for note in NOTES]
        self.note_playing = False
        self.play_requested = False
        self._executor = futures.ThreadPoolExecutor(max_workers=1)
        self._active_sound_lock = threading.Lock()
        self._active_sound: Optional[Sound] = None
        self._cancel_playback = threading.Event()

    def _set_active_sound(self, sound: Optional[Sound]) -> None:
        with self._active_sound_lock:
            self._active_sound = sound

    def _wait_sound_or_cancel(self, note: Sound) -> None:
        while note.is_playing() and not self._cancel_playback.is_set():
            time.sleep(_POLL_S)
        if self._cancel_playback.is_set():
            note.stop()

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
            self._cancel_playback.clear()
            self.note_playing = True
            try:
                self._set_active_sound(note)
                note.play()
                self._wait_sound_or_cancel(note)
            finally:
                self._set_active_sound(None)
                self.note_playing = False
                self.play_requested = False

        self._executor.submit(_play)

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

        def _play():
            self._cancel_playback.clear()
            self.note_playing = True
            try:
                for note in sequence:
                    if self._cancel_playback.is_set():
                        break
                    self._set_active_sound(note)
                    note.play()
                    self._wait_sound_or_cancel(note)
            finally:
                self._set_active_sound(None)
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
            self._cancel_playback.clear()
            self.note_playing = True
            try:
                for note in sequence:
                    if self._cancel_playback.is_set():
                        break
                    self._set_active_sound(note)
                    note.play()
                    self._wait_sound_or_cancel(note)
            finally:
                self._set_active_sound(None)
                self.note_playing = False

        self._executor.submit(_play)

    def stop_all(self):
        """Stop all audio from this controller.

        Uses simpleaudio.stop_all() so buffers started from ephemeral Sound
        instances (jingles) are halted even without a Python reference.
        """
        self._cancel_playback.set()
        try:
            sa.stop_all()
        except Exception:
            pass
        with self._active_sound_lock:
            active = self._active_sound
        if active is not None:
            try:
                active.stop()
            except Exception:
                pass
        for s in self.sounds:
            try:
                s.stop()
            except Exception:
                pass
        self._executor.shutdown(wait=False, cancel_futures=True)
        self._executor = futures.ThreadPoolExecutor(max_workers=1)
        self.note_playing = False
        self.play_requested = False
        self._cancel_playback.clear()

    def wait_done(self, polling_interval=0.05):
        """Blocking function that waits until notes are done playing."""

        time.sleep(polling_interval)
        while self.note_playing:
            time.sleep(polling_interval)
