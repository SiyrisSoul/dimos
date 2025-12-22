# Copyright 2025 Dimensional Inc.
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

"""Audio pipeline framework (audio2).

Modern reactive audio pipeline built on GStreamer and RxPY.

Architecture:
  - Raw PCM F32LE is the standard internal format
  - Inputs decode to raw, outputs encode from raw
  - All data flows through RxPY observables

Basic usage:
    >>> from dimos.stream.audio2 import signal, robotize, speaker
    >>> signal(frequency=440, duration=2.0).pipe(robotize(), speaker()).run()

Text-to-speech:
    >>> from dimos.stream.audio2 import openai_tts, speaker
    >>> openai_tts("Hello world").pipe(speaker()).run()

Modules:
    - input: Audio sources (file, microphone, signal, TTS)
    - operators: Audio transformations (effects, normalizer, vumeter)
    - output: Audio sinks (speaker, network)
    - types: Core data types (AudioEvent, AudioSpec, AudioFormat)
"""

# Modules for high-level usage (optional - requires dimos.agents2)
try:
    from dimos.stream.audio2.module import RemoteSpeechModule, SpeechModule

    _MODULES_AVAILABLE = True
except ImportError:
    _MODULES_AVAILABLE = False
    SpeechModule = None
    RemoteSpeechModule = None

# Input sources
from dimos.stream.audio2.input import (
    Voice,
    file_input,
    microphone,
    openai_tts,
    pyttsx3_tts,
    signal,
)
from dimos.stream.audio2.input.signal import WaveformType

# Operators (effects and processing)
from dimos.stream.audio2.operators import (
    normalizer,
    pitch_shift,
    ring_modulator,
    robotize,
    vumeter,
)

# Output sinks
from dimos.stream.audio2.output import network_output, speaker

# Core types
from dimos.stream.audio2.types import (
    AudioEvent,
    AudioFormat,
    AudioSpec,
    CompressedAudioEvent,
    RawAudioEvent,
)

__all__ = [
    # Modules
    "SpeechModule",
    "RemoteSpeechModule",
    # Inputs
    "file_input",
    "microphone",
    "signal",
    "openai_tts",
    "pyttsx3_tts",
    "Voice",
    "WaveformType",
    # Operators
    "normalizer",
    "vumeter",
    "robotize",
    "pitch_shift",
    "ring_modulator",
    # Outputs
    "speaker",
    "network_output",
    # Types
    "AudioEvent",
    "RawAudioEvent",
    "CompressedAudioEvent",
    "AudioFormat",
    "AudioSpec",
]
