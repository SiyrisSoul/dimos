# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field
import threading

import numpy as np
import sounddevice as sd  # type: ignore[import-untyped]
import whisper  # type: ignore[import-untyped]

from dimos.agents.skills import SpeakSkill
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _load_tts_voice(model_path: str | None, config_path: str | None):
    """Load a piper-tts voice, auto-downloading the default English model if no path is given."""
    try:
        from piper.voice import PiperVoice  # type: ignore[import-not-found]
    except ImportError:
        logger.warning("piper-tts not installed; onboard TTS disabled. Run: pip install piper-tts")
        return None

    if model_path is None:
        try:
            from huggingface_hub import hf_hub_download  # type: ignore[import-not-found]

            model_path = hf_hub_download(
                repo_id="rhasspy/piper-voices",
                filename="en/en_US/lessac/medium/en_US-lessac-medium.onnx",
            )
            config_path = hf_hub_download(
                repo_id="rhasspy/piper-voices",
                filename="en/en_US/lessac/medium/en_US-lessac-medium.onnx.json",
            )
        except Exception as e:
            logger.warning(f"Failed to download piper voice model: {e}; onboard TTS disabled.")
            return None

    try:
        return PiperVoice.load(model_path, config_path=config_path, use_cuda=False)
    except Exception as e:
        logger.warning(f"Failed to load piper voice from {model_path}: {e}; onboard TTS disabled.")
        return None


def _speak_onboard(voice, text: str) -> None:
    """Synthesize and play text with piper-tts (blocking, no network needed)."""
    chunks = list(voice.synthesize(text))
    audio = np.concatenate([c.audio_float_array for c in chunks])
    sd.play(audio, samplerate=chunks[0].sample_rate, blocking=True)


@dataclass
class VoiceConfig(ModuleConfig):
    system_prompt: str | None = SYSTEM_PROMPT
    model: str = "gpt-4o"
    model_fixture: str | None = None
    stt_model: str = "base"  # whisper model size: tiny, base, small, medium, large
    tts_model_path: str | None = None  # path to piper .onnx file; auto-downloads if None
    tts_model_config: str | None = None  # path to piper .onnx.json config
    wake_words: list[str] = field(default_factory=lambda: ["hey robot", "hay robot"])
    wake_silence_cutoff: float = 0.8  # seconds of silence before checking for wake word
    silence_duration: float = 5.0  # seconds of silence that ends a command recording
    sample_rate: int = 16000
    channels: int = 1
    chunk_ms: int = 100  # milliseconds per read slice
    vad_threshold: float = 0.01  # RMS energy threshold for voice activity detection


class Voice(Module):
    default_config: type[VoiceConfig] = VoiceConfig
    config: VoiceConfig

    human_input: Out[str]

    speak_ref: SpeakSkill

    @rpc
    def start(self) -> None:
        super().start()

        # Fast onboard TTS (piper-tts) for low-latency acknowledgment sounds
        voice = _load_tts_voice(self.config.tts_model_path, self.config.tts_model_config)

        stt_model = whisper.load_model(self.config.stt_model)
        logger.info(f"Loaded Whisper STT model: {self.config.stt_model}")

        wake_words = [w.lower() for w in self.config.wake_words]
        wake_silence_cutoff = self.config.wake_silence_cutoff
        silence_duration = self.config.silence_duration
        sample_rate = self.config.sample_rate
        channels = self.config.channels
        chunk_ms = self.config.chunk_ms
        chunk_samples = int(sample_rate * chunk_ms / 1000)
        vad_threshold = self.config.vad_threshold

        def _on_voice_message(string: str) -> None:
            self.human_input.publish(string)

        def _listen_loop() -> None:
            with sd.InputStream(
                samplerate=sample_rate, channels=channels, dtype="float32"
            ) as stream:
                while True:
                    # --- Phase 1: Listen for wake word ---
                    # Accumulate speech in chunk_ms slices; after a short silence, transcribe
                    # the whole utterance and check it for the wake word.
                    logger.info("Listening for wake word...")
                    speech_buf: list[np.ndarray] = []
                    silence_elapsed = 0.0
                    in_speech = False

                    while True:
                        chunk, _ = stream.read(chunk_samples)
                        mono = chunk[:, 0]
                        rms = float(np.sqrt(np.mean(mono**2)))

                        if rms >= vad_threshold:
                            speech_buf.append(mono)
                            silence_elapsed = 0.0
                            in_speech = True
                        elif in_speech:
                            speech_buf.append(mono)
                            silence_elapsed += chunk_ms / 1000

                            if silence_elapsed >= wake_silence_cutoff:
                                audio = np.concatenate(speech_buf)
                                result = stt_model.transcribe(audio, language="en", fp16=False)
                                text = result["text"].strip().lower()

                                if any(w in text for w in wake_words):
                                    logger.info(f"Wake word detected in: '{text}'")
                                    break

                                # Not a wake word — reset and keep listening
                                speech_buf = []
                                silence_elapsed = 0.0
                                in_speech = False

                    # --- Phase 2: Acknowledge immediately with onboard TTS ("yeah") ---
                    # TODO: say "hmm" in a loop via self.speak_ref.speak() while the agent generates a response
                    if voice is not None:
                        threading.Thread(
                            target=_speak_onboard, args=(voice, "yeah"), daemon=True
                        ).start()

                    # --- Phase 3: Record command in chunk_ms slices until silence_duration of silence ---
                    logger.info("Recording command...")
                    command_buf: list[np.ndarray] = []
                    silence_elapsed = 0.0
                    in_speech = False

                    while True:
                        chunk, _ = stream.read(chunk_samples)
                        mono = chunk[:, 0]
                        command_buf.append(mono)
                        rms = float(np.sqrt(np.mean(mono**2)))

                        if rms >= vad_threshold:
                            silence_elapsed = 0.0
                            in_speech = True
                        else:
                            silence_elapsed += chunk_ms / 1000
                            if in_speech and silence_elapsed >= silence_duration:
                                break

                    # --- Phase 4: Transcribe and publish command ---
                    full_audio = np.concatenate(command_buf)
                    result = stt_model.transcribe(full_audio, language="en", fp16=False)
                    command_text = result["text"].strip()
                    logger.info(f"Heard command: '{command_text}'")

                    if command_text:
                        _on_voice_message(command_text)

        threading.Thread(target=_listen_loop, daemon=True, name="voice-listener").start()
