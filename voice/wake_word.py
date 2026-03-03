"""
OpenWakeWord engine wrapper for Ableware wake word detection.

The detector runs against every audio chunk regardless of voice state —
it's cheap enough (~5% CPU on Pi 4) to run continuously.
"""

import logging
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class WakeWordDetector:
    """
    Wraps OpenWakeWord to detect the "Ableware" wake word.

    Feed 16-bit PCM chunks (numpy int16 arrays) via `process_chunk`.
    Returns True when the model confidence exceeds the threshold.
    """

    def __init__(self, model_path: str, threshold: float = 0.5):
        """
        Args:
            model_path: Path to the .onnx wake word model file.
            threshold:  Confidence threshold in [0, 1]. Lower = more sensitive.
        """
        self.model_path = model_path
        self.threshold = threshold
        self._model = None

    def load(self) -> None:
        """Load the ONNX model. Call once before the audio loop starts."""
        try:
            from openwakeword.model import Model  # pylint: disable=import-outside-toplevel

            if not Path(self.model_path).exists():
                raise FileNotFoundError(
                    f"Wake word model not found at {self.model_path!r}. "
                    "Train or download an OpenWakeWord .onnx model and place it there."
                )
            self._model = Model(wakeword_models=[self.model_path], inference_framework="onnx")
            logger.info("Wake word model loaded from %s", self.model_path)
        except ImportError as exc:
            raise ImportError(
                "openwakeword is not installed. Run: pip install openwakeword onnxruntime"
            ) from exc

    def process_chunk(self, pcm_int16: np.ndarray) -> bool:
        """
        Process one audio chunk and return whether the wake word was detected.

        Args:
            pcm_int16: 1-D numpy array of int16 samples at 16 kHz.

        Returns:
            True if confidence >= threshold for this chunk.
        """
        if self._model is None:
            raise RuntimeError("Call WakeWordDetector.load() before process_chunk().")

        # OpenWakeWord expects int16 input
        prediction = self._model.predict(pcm_int16)

        # prediction is a dict {model_name: score}; take the max score
        if not prediction:
            return False

        max_score = max(prediction.values())
        if max_score >= self.threshold:
            logger.debug("Wake word detected (score=%.3f)", max_score)
            return True
        return False

    @property
    def is_loaded(self) -> bool:
        return self._model is not None
