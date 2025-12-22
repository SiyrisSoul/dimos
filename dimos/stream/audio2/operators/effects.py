#!/usr/bin/env python3
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

"""GStreamer-based audio effects operators."""

import threading
import time

import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst
from reactivex import create
from reactivex.abc import ObservableBase

from dimos.stream.audio2.gstreamer import GStreamerPipelineBase
from dimos.stream.audio2.types import AudioEvent, AudioSpec, CompressedAudioEvent, RawAudioEvent
from dimos.stream.audio2.utils import buffer_to_audio_event, validate_pipeline_element
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.effects")


class RobotizeNode(GStreamerPipelineBase):
    """GStreamer-based robotize effect that applies pitch shift and echo."""

    def __init__(
        self,
        pitch_shift: float = 1.1,
        echo_delay: int = 50000000,
        echo_intensity: float = 0.4,
        amplification: float = 4.0,
        modulator_freq: float = 30.0,
        modulator_volume: float = 0.3,
    ):
        super().__init__()

        # Robotize parameters
        self.pitch_shift = pitch_shift
        self.echo_delay = echo_delay
        self.echo_intensity = echo_intensity
        self.amplification = amplification
        self.modulator_freq = modulator_freq
        self.modulator_volume = modulator_volume

        # Pipeline elements
        self._appsrc = None
        self._appsink = None
        self._pitch = None
        self._echo = None

        # Input/output format tracking
        self._input_format = None
        self._output_rate = None
        self._output_channels = None

        # Threading
        self._pull_thread = None
        self._observer = None

    def _create_pipeline(self):
        """Create the robotize effect pipeline."""
        # Simplified pipeline without ring modulator for now - just effects + amplification
        # Pipeline: appsrc → decodebin → audioconvert → pitch → audioecho → amplify → appsink
        pipeline_str = (
            f"appsrc name=src ! "
            f"queue ! "
            f"decodebin ! "
            f"audioconvert ! "
            f"pitch name=pitch pitch={self.pitch_shift} ! "
            f"audioecho name=echo delay={self.echo_delay} intensity={self.echo_intensity} feedback={self.echo_intensity} ! "
            f"audioamplify name=amplify amplification={self.amplification} ! "
            f"audioconvert ! "
            f"audio/x-raw,format=F32LE ! "
            f"appsink name=sink emit-signals=true sync=false"
        )

        logger.debug(f"Creating robotize pipeline: {pipeline_str}")
        self._pipeline = Gst.parse_launch(pipeline_str)

        # Get elements
        self._appsrc = validate_pipeline_element(self._pipeline, "src")
        self._appsink = validate_pipeline_element(self._pipeline, "sink")
        self._pitch = validate_pipeline_element(self._pipeline, "pitch")
        self._echo = validate_pipeline_element(self._pipeline, "echo")

        # Configure appsrc
        self._appsrc.set_property("is-live", False)
        self._appsrc.set_property("format", Gst.Format.TIME)
        self._appsrc.set_property("block", False)

        # Configure appsink
        self._appsink.set_property("emit-signals", True)
        self._appsink.set_property("sync", False)

        # Set up bus
        self._setup_bus(self._pipeline)

    def _push_event_to_pipeline(self, event: AudioEvent):
        """Push an AudioEvent to the pipeline via appsrc."""

        # Set caps on first event
        if self._input_format is None:
            self._input_format = AudioSpec(
                format=event.format, sample_rate=event.sample_rate, channels=event.channels
            )
            caps_str = self._input_format.to_gst_caps_string()
            if "layout=" not in caps_str:
                caps_str += ",layout=interleaved"
            caps = Gst.Caps.from_string(caps_str)
            self._appsrc.set_property("caps", caps)
            logger.info(f"Set input caps: {caps_str}")

        # Create buffer
        if isinstance(event, RawAudioEvent):
            if event.channels > 1:
                data = np.ascontiguousarray(event.data)
            else:
                data = event.data
            buffer = Gst.Buffer.new_wrapped(data.tobytes())
        elif isinstance(event, CompressedAudioEvent):
            buffer = Gst.Buffer.new_wrapped(event.data)
        else:
            logger.warning(f"Unknown event type: {type(event)}")
            return

        # Set timestamp
        buffer.pts = int(event.timestamp * Gst.SECOND)

        # Push to pipeline
        ret = self._appsrc.emit("push-buffer", buffer)
        if ret != Gst.FlowReturn.OK:
            logger.warning(f"Failed to push buffer: {ret}")

    def _pull_from_pipeline(self):
        """Pull processed samples from appsink and emit as AudioEvents."""
        logger.debug("Pull thread starting")

        buffer_count = 0
        try:
            while self._running:
                sample = self._appsink.emit("pull-sample")
                if sample is None:
                    if not self._running:
                        break
                    time.sleep(0.001)
                    continue

                buffer = sample.get_buffer()
                if buffer is None:
                    continue

                # Parse caps on first sample
                if self._output_rate is None:
                    caps = sample.get_caps()
                    if caps:
                        structure = caps.get_structure(0)
                        self._output_rate = structure.get_int("rate")[1]
                        self._output_channels = structure.get_int("channels")[1]
                        logger.info(
                            f"Output format: {self._output_rate}Hz, {self._output_channels}ch"
                        )

                # Convert to AudioEvent
                from dimos.stream.audio2.types import AudioFormat

                output_spec = AudioSpec(
                    format=AudioFormat.PCM_F32LE,
                    sample_rate=self._output_rate,
                    channels=self._output_channels,
                )

                # Extract timestamp from buffer PTS (preserve timing from input)
                buffer_timestamp = None
                if buffer.pts != Gst.CLOCK_TIME_NONE:
                    buffer_timestamp = buffer.pts / Gst.SECOND

                event = buffer_to_audio_event(
                    buffer=buffer,
                    spec=output_spec,
                    detected_rate=self._output_rate,
                    detected_channels=self._output_channels,
                    timestamp=buffer_timestamp,
                )

                buffer_count += 1

                if self._observer:
                    self._observer.on_next(event)

        except Exception as e:
            logger.error(f"Pull thread error: {e}")
            if self._observer:
                self._observer.on_error(e)

        finally:
            logger.info(f"Pull thread exiting (processed {buffer_count} buffers)")

    def process_observable(self, source: ObservableBase) -> ObservableBase:
        """Process an observable of AudioEvents through the robotize effect."""

        def subscribe(observer, scheduler=None):
            self._observer = observer

            try:
                # Ensure mainloop is running
                self._ensure_pipeline_ready()
                self._create_pipeline()

                # Start pipeline
                ret = self._pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("Failed to start pipeline")

                # Start pull thread
                self._pull_thread = threading.Thread(
                    target=self._pull_from_pipeline, daemon=True, name="Robotize-pull"
                )
                self._pull_thread.start()

                logger.info(
                    f"GStreamer robotize started "
                    f"(pitch={self.pitch_shift}, echo_delay={self.echo_delay}ns)"
                )

                # Subscribe to source and push events to pipeline
                def on_next(event):
                    self._push_event_to_pipeline(event)

                def on_error(error):
                    logger.error(f"Source error: {error}")
                    self._cleanup_pipeline()
                    observer.on_error(error)

                def on_completed():
                    logger.info("Source completed, sending EOS")
                    if self._appsrc:
                        self._appsrc.emit("end-of-stream")
                    # Wait a bit for pipeline to finish processing
                    time.sleep(0.1)
                    self._running = False
                    if self._pull_thread:
                        self._pull_thread.join(timeout=2.0)

                    # Cleanup pipeline BEFORE calling observer.on_completed()
                    # This releases the mainloop reference so downstream operators
                    # (like network_output) can finish their EOS processing
                    self._cleanup_pipeline()

                    # Now notify observer
                    observer.on_completed()

                source.subscribe(on_next, on_error, on_completed, scheduler=scheduler)

            except Exception as e:
                observer.on_error(e)
                self._cleanup_pipeline()

            def dispose():
                logger.info("Disposing GStreamer robotize effect")
                self._running = False
                if self._pipeline:
                    self._pipeline.set_state(Gst.State.NULL)
                if self._pull_thread:
                    self._pull_thread.join(timeout=2.0)
                self._cleanup_pipeline()

            from reactivex import disposable

            return disposable.Disposable(dispose)

        return create(subscribe)


def robotize(**kwargs):
    """Create a GStreamer-based robotize effect operator.

    This operator applies robotic audio effects using pitch shifting, echo, amplification,
    and ring modulation (sawtooth wave mixed in). Works with both raw and compressed audio
    formats by automatically decoding/encoding.

    Args:
        pitch_shift: Pitch shift multiplier (default: 1.1)
        echo_delay: Echo delay in nanoseconds (default: 50ms = 50000000ns)
        echo_intensity: Echo feedback intensity 0.0-1.0 (default: 0.4)
        amplification: Amplification factor after effects (default: 4.0)
        modulator_freq: Ring modulator frequency in Hz (default: 30.0)
        modulator_volume: Ring modulator mix volume 0.0-1.0 (default: 0.3)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Make audio sound like a robot
        signal(frequency=440).pipe(
            robotize(),
            speaker()
        ).run()

        # Custom robot effect with more modulation
        file_input("voice.wav").pipe(
            robotize(pitch_shift=1.2, modulator_freq=40, modulator_volume=0.5),
            network_output()
        ).run()
    """

    node = RobotizeNode(**kwargs)

    def _robotize(source: ObservableBase) -> ObservableBase:
        return node.process_observable(source)

    return _robotize
