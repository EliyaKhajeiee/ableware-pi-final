"""
Audio feedback module for the Ableware Pi voice client.

Generates beep tones via the Pi HAT speaker using sounddevice,
and optional TTS announcements via pyttsx3.
"""

import asyncio
import logging
import threading
from typing import Optional

import numpy as np
import sounddevice as sd

logger = logging.getLogger(__name__)


def _generate_tone(freq: float, duration_ms: int, sample_rate: int = 16000) -> np.ndarray:
    """Generate a sine-wave tone as a float32 numpy array."""
    t = np.linspace(0, duration_ms / 1000, int(sample_rate * duration_ms / 1000), endpoint=False)
    # Apply a short linear fade-in/out (5ms) to eliminate clicks
    fade_samples = int(sample_rate * 0.005)
    tone = (np.sin(2 * np.pi * freq * t) * 0.6).astype(np.float32)
    fade = np.linspace(0, 1, fade_samples)
    if len(tone) > 2 * fade_samples:
        tone[:fade_samples] *= fade
        tone[-fade_samples:] *= fade[::-1]
    return tone


def _play_blocking(audio: np.ndarray, sample_rate: int) -> None:
    """Play audio synchronously (called from a thread pool)."""
    sd.play(audio, samplerate=sample_rate, blocking=True)


class AudioFeedback:
    """
    Plays beep tones and TTS on the Pi HAT speaker.

    All playback is non-blocking from the caller's perspective — audio is
    dispatched to a daemon thread so the asyncio event loop is never stalled.
    """

    def __init__(
        self,
        wake_freq: int = 880,
        confirm_freq: int = 1047,
        error_freq: int = 220,
        beep_duration_ms: int = 150,
        sample_rate: int = 16000,
    ):
        self.sample_rate = sample_rate
        self.beep_duration_ms = beep_duration_ms

        # Pre-generate tones at init so playback has no latency
        self._wake_tone = _generate_tone(wake_freq, beep_duration_ms, sample_rate)
        self._confirm_tone = _generate_tone(confirm_freq, beep_duration_ms, sample_rate)
        self._error_tone = _generate_tone(error_freq, beep_duration_ms, sample_rate)

        # Double-beep: two tones with a short gap
        gap = np.zeros(int(sample_rate * 0.05), dtype=np.float32)
        self._wake_double = np.concatenate([self._wake_tone, gap, self._wake_tone])

        # Optional TTS engine — initialised lazily to avoid import lag
        self._tts_engine = None
        self._tts_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public async API
    # ------------------------------------------------------------------

    async def play_wake(self) -> None:
        """Double-beep — played when the wake word is detected."""
        await self._play_async(self._wake_double)

    async def play_confirm(self) -> None:
        """Single confirm beep — played after a command is dispatched."""
        await self._play_async(self._confirm_tone)

    async def play_error(self) -> None:
        """Low error beep — played on timeout or connection failure."""
        await self._play_async(self._error_tone)

    async def speak(self, text: str) -> None:
        """Speak a short phrase via pyttsx3 TTS (non-blocking)."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._speak_blocking, text)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    async def _play_async(self, audio: np.ndarray) -> None:
        """Run sounddevice playback in a thread-pool executor."""
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(None, _play_blocking, audio, self.sample_rate)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Audio playback error: %s", exc)

    def _speak_blocking(self, text: str) -> None:
        """Initialise and run pyttsx3 TTS synchronously."""
        try:
            import pyttsx3  # pylint: disable=import-outside-toplevel

            with self._tts_lock:
                if self._tts_engine is None:
                    self._tts_engine = pyttsx3.init()
                    self._tts_engine.setProperty("rate", 160)
                engine = self._tts_engine
                engine.say(text)
                engine.runAndWait()
        except Exception as exc:  # noqa: BLE001
            logger.warning("TTS error: %s", exc)
