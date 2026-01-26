# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.encoders import (
    JpegEncoderMixin,
    LCMEncoderMixin,
    PickleEncoderMixin,
)
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.lcmservice import (
    LCMConfig,
    LCMService,
    autoconf,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable
    import re
    import threading

    from dimos.msgs import DimosMsg

logger = setup_logger()


@dataclass
class Topic:
    topic: str = ""
    lcm_type: type[DimosMsg] | None = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.msg_name}"


class LCMPubSubBase(LCMService, PubSub[Topic, Any]):
    """LCM-based PubSub with native regex subscription support.

    LCM natively supports regex patterns in subscribe(), so we implement
    RegexSubscribable directly without needing discovery-based fallback.
    """

    default_config = LCMConfig
    _stop_event: threading.Event
    _thread: threading.Thread | None

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

    def publish(self, topic: Topic, message: bytes) -> None:
        """Publish a message to the specified channel."""
        if self.l is None:
            logger.error("Tried to publish after LCM was closed")
            return

        self.l.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        if self.l is None:
            logger.error("Tried to subscribe after LCM was closed")

            def noop() -> None:
                pass

            return noop

        lcm_subscription = self.l.subscribe(str(topic), lambda _, msg: callback(msg, topic))

        # Set queue capacity to 10000 to handle high-volume bursts
        lcm_subscription.set_queue_capacity(10000)

        def unsubscribe() -> None:
            if self.l is None:
                return
            self.l.unsubscribe(lcm_subscription)

        return unsubscribe


class LCM(
    LCMEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...


class PickleLCM(
    PickleEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...


class JpegLCM(
    JpegEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...


__all__ = [
    "LCM",
    "JpegLCM",
    "LCMEncoderMixin",
    "LCMPubSubBase",
    "PickleLCM",
    "autoconf",
]
