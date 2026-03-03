"""
Ableware Pi — Voice Client Entry Point.

Usage:
    python3 voice/main.py [--config /path/to/config.yaml]

System dependencies (install before running):
    sudo apt install espeak-ng portaudio19-dev
"""

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path

import yaml

# Allow running from the voice/ directory directly
sys.path.insert(0, os.path.dirname(__file__))

from audio_feedback import AudioFeedback
from recognizer import CommandRecognizer
from voice_loop import VoiceLoop
from wake_word import WakeWordDetector
from ws_client import PiWebSocketClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("ableware.pi")


def _load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


async def main(config_path: str) -> None:
    cfg = _load_config(config_path)

    server_ip = cfg["server"]["ip"]
    hub_port = cfg["server"]["hub_port"]
    ws_uri = f"ws://{server_ip}:{hub_port}/ws/pi"

    voice_cfg = cfg["voice"]
    feedback_cfg = cfg["feedback"]

    # ---- Build components ----
    feedback = AudioFeedback(
        wake_freq=feedback_cfg["wake_beep_freq"],
        confirm_freq=feedback_cfg["confirm_beep_freq"],
        error_freq=feedback_cfg["error_beep_freq"],
        beep_duration_ms=feedback_cfg["beep_duration_ms"],
        sample_rate=voice_cfg["sample_rate"],
    )

    wake_detector = WakeWordDetector(
        model_path=voice_cfg["wake_word_model"],
        threshold=voice_cfg["wake_word_threshold"],
    )

    vosk_model_path = os.path.join(os.path.dirname(__file__), voice_cfg["vosk_model"])
    recognizer = CommandRecognizer(
        model_path=vosk_model_path,
        sample_rate=voice_cfg["sample_rate"],
    )

    ws_client = PiWebSocketClient(uri=ws_uri)

    # ---- Check wake word model availability ----
    ww_model_path = os.path.join(os.path.dirname(__file__), voice_cfg["wake_word_model"])
    bypass_wake_word = not Path(ww_model_path).exists()
    if bypass_wake_word:
        logger.warning(
            "Wake word model not found at %s — running in BYPASS mode (always listening).",
            ww_model_path,
        )
    else:
        logger.info("Loading wake word model …")
        wake_detector.load()

    voice_loop = VoiceLoop(
        wake_detector=wake_detector if not bypass_wake_word else None,
        recognizer=recognizer,
        feedback=feedback,
        ws_client=ws_client,
        sample_rate=voice_cfg["sample_rate"],
        chunk_size=voice_cfg["chunk_size"],
        listen_timeout_s=voice_cfg["listen_timeout_s"],
        bypass_wake_word=bypass_wake_word,
    )

    logger.info("Loading Vosk ASR model …")
    recognizer.load()

    # ---- Run everything concurrently ----
    logger.info("Starting Ableware Pi voice client. WS target: %s", ws_uri)
    await asyncio.gather(
        ws_client.start(),     # background reconnect loop
        voice_loop.run(),      # foreground audio loop
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ableware Pi voice client")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "..", "config.yaml"),
        help="Path to config.yaml",
    )
    args = parser.parse_args()

    try:
        asyncio.run(main(args.config))
    except KeyboardInterrupt:
        logger.info("Ableware Pi client stopped.")
