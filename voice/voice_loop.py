"""
4-state voice control loop for the Ableware Pi client.

State machine:
    IDLE ──[wake word]──▶ WAKE_DETECTED ──[double beep]──▶ LISTENING
      ▲                                                          │
      │◀──[timeout / send success]──── SENDING ◀──[command]─────┘
"""

import asyncio
import logging
import time
from enum import Enum, auto
from typing import Optional

import numpy as np
import sounddevice as sd

logger = logging.getLogger(__name__)


class VoiceState(Enum):
    IDLE = auto()
    WAKE_DETECTED = auto()
    LISTENING = auto()
    SENDING = auto()


class VoiceLoop:
    """
    Manages the full voice pipeline: audio capture → wake word → ASR → WS send.

    All heavy I/O (audio, network) is async or run in executors so the event
    loop stays responsive.
    """

    def __init__(
        self,
        wake_detector,          # WakeWordDetector or None
        recognizer,             # CommandRecognizer
        feedback,               # AudioFeedback
        ws_client,              # PiWebSocketClient
        sample_rate: int = 16000,
        chunk_size: int = 1280,
        listen_timeout_s: float = 3.0,
        bypass_wake_word: bool = False,
    ):
        self.wake_detector = wake_detector
        self.recognizer = recognizer
        self.feedback = feedback
        self.ws_client = ws_client
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.listen_timeout_s = listen_timeout_s
        self.bypass_wake_word = bypass_wake_word

        self._state = VoiceState.IDLE
        self._audio_queue: asyncio.Queue = asyncio.Queue(maxsize=50)
        self._running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Main entry point — starts the audio capture thread and the processing loop.
        Run this as the main coroutine (or await it in main.py).
        """
        self._running = True
        loop = asyncio.get_event_loop()

        # sounddevice callback pushes chunks into the asyncio queue
        def _audio_callback(indata, frames, time_info, status):
            if status:
                logger.warning("Audio status: %s", status)
            # Copy to avoid the buffer being overwritten before we process it
            chunk = indata[:, 0].copy()
            try:
                loop.call_soon_threadsafe(
                    self._audio_queue.put_nowait,
                    (chunk * 32767).astype(np.int16),
                )
            except asyncio.QueueFull:
                pass  # Drop chunk if we're behind — better than blocking the stream

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            blocksize=self.chunk_size,
            callback=_audio_callback,
        ):
            if self.bypass_wake_word:
                logger.info("Audio stream started. Wake word BYPASSED — listening continuously.")
            else:
                logger.info("Audio stream started. Listening for wake word 'Ableware' …")
            await self._process_loop()

    async def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    async def _process_loop(self) -> None:
        """Continuously pull chunks from the queue and advance the state machine."""
        while self._running:
            try:
                chunk = await asyncio.wait_for(self._audio_queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue

            if self._state == VoiceState.IDLE:
                await self._handle_idle(chunk)
            elif self._state == VoiceState.LISTENING:
                await self._handle_listening(chunk)
            # WAKE_DETECTED and SENDING are transient — handled inline

    async def _handle_idle(self, chunk: np.ndarray) -> None:
        """Check chunk for wake word; transition to WAKE_DETECTED if found."""
        if self.bypass_wake_word:
            # No wake word model — transition straight to listening
            await self._transition_to_listening()
            return
        if self.wake_detector.process_chunk(chunk):
            logger.info("Wake word detected!")
            self._state = VoiceState.WAKE_DETECTED
            await self._transition_to_listening()

    async def _transition_to_listening(self) -> None:
        """Play double-beep, reset ASR, enter LISTENING state."""
        await self.feedback.play_wake()
        self.recognizer.reset()
        self._state = VoiceState.LISTENING
        self._listen_start = time.monotonic()
        logger.info("Listening for command (%.1fs window) …", self.listen_timeout_s)

    async def _handle_listening(self, chunk: np.ndarray) -> None:
        """Feed chunk to ASR; on result or timeout → SENDING or IDLE."""
        elapsed = time.monotonic() - self._listen_start

        # Feed PCM bytes to Vosk
        command = self.recognizer.process_chunk(chunk.tobytes())

        if command is None and elapsed >= self.listen_timeout_s:
            # Try flushing any partial hypothesis
            command = self.recognizer.finalize()

        if command:
            await self._dispatch_command(command)
        elif elapsed >= self.listen_timeout_s:
            logger.info("Listen window expired with no command.")
            await self.feedback.play_error()
            self._state = VoiceState.IDLE

    async def _dispatch_command(self, command: str) -> None:
        """Send command via WebSocket and play confirm/error beep."""
        self._state = VoiceState.SENDING
        logger.info("Dispatching command: %s", command)

        success = await self.ws_client.send_command(command, source="voice")
        if success:
            await self.feedback.play_confirm()
        else:
            await self.feedback.play_error()
            logger.warning("Command queued (not connected): %s", command)

        self._state = VoiceState.IDLE
