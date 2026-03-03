"""
Grammar-constrained Vosk speech recognizer.

Only accepts: START, STOP, UP, DOWN, LEFT, RIGHT.
The [unk] token catches everything else and is discarded — eliminates
~95% of false positives from background noise and off-command speech.
"""

import json
import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

VALID_COMMANDS = {"START", "STOP", "UP", "DOWN", "LEFT", "RIGHT"}

# Vosk grammar: lowercase list + [unk] to absorb unknown speech
COMMAND_GRAMMAR = json.dumps(["start", "stop", "up", "down", "left", "right", "[unk]"])


class CommandRecognizer:
    """
    Vosk-based speech recognizer constrained to the six Ableware commands.

    Usage:
        rec = CommandRecognizer(model_path)
        rec.load()
        rec.reset()
        command = rec.process_chunk(pcm_int16_bytes)  # call per audio chunk
        if command:
            handle(command)
    """

    def __init__(self, model_path: str, sample_rate: int = 16000):
        self.model_path = model_path
        self.sample_rate = sample_rate
        self._recognizer = None

    def load(self) -> None:
        """Load the Vosk model. Call once before the audio loop starts."""
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel  # pylint: disable=import-outside-toplevel

            SetLogLevel(-1)  # Suppress verbose Vosk output

            model_dir = Path(self.model_path)
            if not model_dir.exists():
                raise FileNotFoundError(
                    f"Vosk model not found at {self.model_path!r}. "
                    "Download from https://alphacephei.com/vosk/models and extract here."
                )

            vosk_model = Model(str(model_dir))
            self._recognizer = KaldiRecognizer(vosk_model, self.sample_rate, COMMAND_GRAMMAR)
            self._recognizer.SetWords(False)
            logger.info("Vosk model loaded from %s", self.model_path)
        except ImportError as exc:
            raise ImportError("vosk is not installed. Run: pip install vosk") from exc

    def reset(self) -> None:
        """Reset recognizer state. Call before each new listen window."""
        if self._recognizer is not None:
            self._recognizer.Reset()

    def process_chunk(self, pcm_bytes: bytes) -> Optional[str]:
        """
        Feed a chunk of raw PCM bytes (int16 LE) and return a command if recognised.

        Returns:
            Uppercase command string ("UP", "DOWN", etc.) or None if not yet decided.
            Returns None also if the result is [unk] or empty.
        """
        if self._recognizer is None:
            raise RuntimeError("Call CommandRecognizer.load() before process_chunk().")

        if self._recognizer.AcceptWaveform(pcm_bytes):
            result = json.loads(self._recognizer.Result())
            text = result.get("text", "").strip().upper()
            return text if text in VALID_COMMANDS else None

        # Partial result — not ready yet
        return None

    def finalize(self) -> Optional[str]:
        """
        Flush any buffered audio and return a final result.

        Call this when the listen window times out.
        """
        if self._recognizer is None:
            return None
        result = json.loads(self._recognizer.FinalResult())
        text = result.get("text", "").strip().upper()
        return text if text in VALID_COMMANDS else None

    @property
    def is_loaded(self) -> bool:
        return self._recognizer is not None
