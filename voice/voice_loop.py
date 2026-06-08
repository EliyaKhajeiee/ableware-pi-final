"""
4-state voice control loop for the Ableware Pi client.

State machine:
    IDLE ──[wake word]──▶ WAKE_DETECTED ──[double beep]──▶ LISTENING
      ▲                                                          │
      │◀──[timeout / send success]──── SENDING ◀──[command]─────┘

Audio backends:
    sounddevice  — default; works when PortAudio can open the device
    arecord      — subprocess fallback; bypasses PortAudio for stubborn ALSA devices
                   (set audio_backend: "arecord" in config.yaml)
"""

import asyncio
import logging
import time
from enum import Enum, auto
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class VoiceState(Enum):
    IDLE = auto()
    WAKE_DETECTED = auto()
    LISTENING = auto()
    SENDING = auto()


class VoiceLoop:
    """
    Manages the full voice pipeline: audio capture → wake word → ASR → WS send.
    """

    def __init__(
        self,
        wake_detector,
        recognizer,
        feedback,
        ws_client,
        sample_rate: int = 16000,
        chunk_size: int = 1280,
        listen_timeout_s: float = 3.0,
        bypass_wake_word: bool = False,
        device=None,
        channels: int = 1,
        audio_backend: str = "sounddevice",   # "sounddevice" or "arecord"
        arecord_device: str = "hw:2,0",
        arecord_rate: int = 44100,            # capture rate; resampled to sample_rate
    ):
        self.wake_detector = wake_detector
        self.recognizer = recognizer
        self.feedback = feedback
        self.ws_client = ws_client
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.listen_timeout_s = listen_timeout_s
        self.bypass_wake_word = bypass_wake_word
        self.device = device
        self.channels = channels
        self.audio_backend = audio_backend
        self.arecord_device = arecord_device
        self.arecord_rate = arecord_rate

        self._state = VoiceState.IDLE
        self._audio_queue: asyncio.Queue = asyncio.Queue(maxsize=50)
        self._running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def run(self) -> None:
        self._running = True
        if self.audio_backend == "arecord":
            await self._run_arecord()
        else:
            await self._run_sounddevice()

    async def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------
    # Audio backends
    # ------------------------------------------------------------------

    async def _run_sounddevice(self) -> None:
        import sounddevice as sd

        loop = asyncio.get_event_loop()

        def _audio_callback(indata, frames, time_info, status):
            if status:
                logger.warning("Audio status: %s", status)
            chunk = indata[:, 0].copy()
            try:
                loop.call_soon_threadsafe(
                    self._audio_queue.put_nowait,
                    (chunk * 32767).astype(np.int16),
                )
            except asyncio.QueueFull:
                pass

        stream_kwargs = dict(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype="float32",
            blocksize=self.chunk_size,
            callback=_audio_callback,
        )
        if self.device is not None:
            stream_kwargs["device"] = self.device

        with sd.InputStream(**stream_kwargs):
            logger.info("sounddevice stream open (device=%s, %dHz, %dch).",
                        self.device, self.sample_rate, self.channels)
            await self._process_loop()

    async def _run_arecord(self) -> None:
        """Capture audio via arecord subprocess — bypasses PortAudio entirely."""
        cmd = [
            "arecord",
            "-D", self.arecord_device,
            "-r", str(self.arecord_rate),
            "-c", str(self.channels),
            "-f", "S16_LE",
            "-t", "raw",
        ]
        logger.info("Starting arecord: %s", " ".join(cmd))

        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )

        # Read stderr in background so it doesn't block stdout
        asyncio.ensure_future(self._log_arecord_stderr(proc))

        bytes_per_chunk = self.chunk_size * self.channels * 2  # int16 = 2 bytes
        need_resample = self.arecord_rate != self.sample_rate

        logger.info("arecord stream open (%s, %dHz→%dHz, %dch).",
                    self.arecord_device, self.arecord_rate, self.sample_rate, self.channels)

        process_task = asyncio.ensure_future(self._process_loop())

        try:
            while self._running:
                data = await proc.stdout.read(bytes_per_chunk)
                if not data:
                    logger.error("arecord process ended unexpectedly.")
                    break

                pcm = np.frombuffer(data, dtype=np.int16)

                # De-interleave: take channel 0 only
                if self.channels > 1:
                    pcm = pcm[::self.channels]

                # Resample if capture rate differs from Vosk's expected rate
                if need_resample:
                    old_len = len(pcm)
                    new_len = int(old_len * self.sample_rate / self.arecord_rate)
                    if new_len > 0:
                        indices = np.linspace(0, old_len - 1, new_len)
                        pcm = np.interp(indices, np.arange(old_len),
                                        pcm.astype(np.float32)).astype(np.int16)

                try:
                    self._audio_queue.put_nowait(pcm)
                except asyncio.QueueFull:
                    pass  # drop stale audio rather than blocking
        finally:
            process_task.cancel()
            proc.terminate()
            await proc.wait()

    async def _log_arecord_stderr(self, proc) -> None:
        async for line in proc.stderr:
            text = line.decode(errors="replace").strip()
            if text:
                logger.debug("arecord stderr: %s", text)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    async def _process_loop(self) -> None:
        while self._running:
            try:
                chunk = await asyncio.wait_for(self._audio_queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue

            if self._state == VoiceState.IDLE:
                await self._handle_idle(chunk)
            elif self._state == VoiceState.LISTENING:
                await self._handle_listening(chunk)

    async def _handle_idle(self, chunk: np.ndarray) -> None:
        if self.bypass_wake_word:
            await self._transition_to_listening()
            return
        if self.wake_detector.process_chunk(chunk):
            logger.info("Wake word detected!")
            self._state = VoiceState.WAKE_DETECTED
            await self._transition_to_listening()

    async def _transition_to_listening(self) -> None:
        await self.feedback.play_wake()
        self.recognizer.reset()
        self._state = VoiceState.LISTENING
        self._listen_start = time.monotonic()
        logger.info("Listening for command (%.1fs window) …", self.listen_timeout_s)

    async def _handle_listening(self, chunk: np.ndarray) -> None:
        elapsed = time.monotonic() - self._listen_start
        command = self.recognizer.process_chunk(chunk.tobytes())

        if command is None and elapsed >= self.listen_timeout_s:
            command = self.recognizer.finalize()

        if command:
            await self._dispatch_command(command)
        elif elapsed >= self.listen_timeout_s:
            logger.info("Listen window expired with no command.")
            await self.feedback.play_error()
            self._state = VoiceState.IDLE

    async def _dispatch_command(self, command: str) -> None:
        self._state = VoiceState.SENDING
        logger.info("Dispatching command: %s", command)
        success = await self.ws_client.send_command(command, source="voice")
        if success:
            await self.feedback.play_confirm()
        else:
            await self.feedback.play_error()
            logger.warning("Command queued (not connected): %s", command)
        self._state = VoiceState.IDLE
