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

"""Rerun bridge for logging pubsub messages with to_rerun() methods."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import TYPE_CHECKING, Any, Literal, Protocol, TypeAlias, runtime_checkable

from reactivex.disposable import Disposable
from toolz import pipe  # type: ignore[import-untyped]

from dimos.core import Module, rpc
from dimos.core.module import ModuleConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    from collections.abc import Callable

    from rerun._baseclasses import Archetype

    from dimos.protocol.pubsub.spec import SubscribeAllCapable

# to_rerun() can return a single archetype or a list of (entity_path, archetype) tuples
RerunData: TypeAlias = "Archetype | list[tuple[str, Archetype]]"


@runtime_checkable
class RerunConvertible(Protocol):
    """Protocol for messages that can be converted to Rerun data."""

    def to_rerun(self) -> RerunData: ...


ViewerMode = Literal["native", "web", "none"]


# Notes on this system
#
# In the future it would be nice if modules can annotate their individual OUTs with (general or rerun specific)
# hints related to their visualization
#
# so stuff like color, update frequency etc (some Image needs to be rendered on the 3d floor like occupancy grid)
# some other image is an image to be streamed into a specific 2D view etc.
#
# to achieve this we'd feed a full blueprint into the rerun bridge.
# rerun bridge can then inspect all transports used, all modules with their outs,
# automatically spy an all the transports and read visualization hints
#
# this is the correct implementation.
# temporarily we are using these "sideloading" transforms={} to define custom to_rerun methods for specific topics
# as well as pubsubs to specify which protocols to listen to.


@dataclass
class Config(ModuleConfig):
    """Configuration for RerunBridgeModule."""

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(
        default_factory=lambda: [LCM(autoconf=True)]
    )

    transforms: dict[Glob | str, Callable[[Any], Archetype]] = field(default_factory=dict)

    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None
    viewer_mode: ViewerMode = "native"
    memory_limit: str = "25%"


class RerunBridgeModule(Module):
    """Bridge that logs messages from pubsubs to Rerun.

    Spawns its own Rerun viewer and subscribes to all topics on each provided
    pubsub. Any message that has a to_rerun() method is automatically logged.

    Example:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        lcm = LCM(autoconf=True)
        bridge = RerunBridgeModule(pubsubs=[lcm])
        bridge.start()
        # All messages with to_rerun() are now logged to Rerun
        bridge.stop()
    """

    default_config = Config
    config: Config

    @lru_cache(maxsize=256)
    def _transform_for_entity_path(self, entity_path: str) -> Callable[[Any], RerunData | None]:
        """Return a composed transform for the entity path.

        Chains matching transforms from config, ending with obligatory_transform
        which handles .to_rerun() or passes through Archetypes.
        """
        from rerun._baseclasses import Archetype

        matches = [
            fn
            for pattern, fn in self.config.transforms.items()
            if pattern_matches(pattern, entity_path)
        ]

        def obligatory_transform(msg: Any) -> RerunData | None:
            if isinstance(msg, Archetype):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        return lambda msg: pipe(msg, *matches, obligatory_transform)

    def _get_entity_path(self, topic: Any) -> str:
        """Convert a topic to a Rerun entity path."""
        if self.config.topic_to_entity:
            return self.config.topic_to_entity(topic)

        # Default: use topic.name if available (LCM Topic), else str
        topic_str = getattr(topic, "name", None) or str(topic)
        # Strip everything after # (LCM topic suffix)
        topic_str = topic_str.split("#")[0]
        return f"{self.config.entity_prefix}/{topic_str}"

    def _on_message(self, msg: Any, topic: Any) -> None:
        import rerun as rr

        """Handle incoming message - log to rerun."""
        # convert a potentially complex topic object into an str rerun entity path
        entity_path: str = self._get_entity_path(topic)

        # apply transforms (including obligatory_transform which handles .to_rerun())
        rerun_data: RerunData | None = self._transform_for_entity_path(entity_path)(msg)

        # transforms can also supress logging by returning None
        if not rerun_data:
            return

        # TFMessage returns list of (entity_path, transform) tuples
        if isinstance(rerun_data, list):
            for entity_path, archetype in rerun_data:
                rr.log(entity_path, archetype)
        else:
            rr.log(entity_path, rerun_data)

    @rpc
    def start(self) -> None:
        import rerun as rr

        super().start()

        # Initialize and spawn Rerun viewer
        rr.init("dimos-bridge")
        if self.config.viewer_mode == "native":
            rr.spawn(connect=True, memory_limit=self.config.memory_limit)
        elif self.config.viewer_mode == "web":
            rr.serve_web_viewer(open_browser=True)
        # "none" - just init, no viewer (connect externally)

        # Start pubsubs and subscribe to all messages
        for pubsub in self.config.pubsubs:
            logger.info(f"bridge listening on {pubsub.__class__.__name__}")
            if hasattr(pubsub, "start"):
                pubsub.start()  # type: ignore[union-attr]
            unsub = pubsub.subscribe_all(self._on_message)
            self._disposables.add(Disposable(unsub))

        # Add pubsub stop as disposable
        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "stop"):
                self._disposables.add(Disposable(pubsub.stop))  # type: ignore[union-attr]

    @rpc
    def stop(self) -> None:
        super().stop()


def main() -> None:
    """CLI entry point for rerun-bridge."""
    import argparse
    import signal

    parser = argparse.ArgumentParser(description="Rerun bridge for LCM messages")
    parser.add_argument(
        "--viewer-mode",
        choices=["native", "web", "none"],
        default="native",
        help="Viewer mode: native (desktop), web (browser), none (headless)",
    )
    parser.add_argument(
        "--memory-limit",
        default="25%",
        help="Memory limit for Rerun viewer (e.g., '4GB', '16GB', '25%%')",
    )
    args = parser.parse_args()

    bridge = RerunBridgeModule(
        viewer_mode=args.viewer_mode,
        memory_limit=args.memory_limit,
        # any pubsub that supports subscribe_all and topic that supports str(topic)
        # is acceptable here
        pubsubs=[LCM(autoconf=True)],
        # defines transforms for rerun entity paths
        transforms={
            Glob("debug_navigation"): lambda image: image.to_rerun(mode="floor"),
        },
    )

    bridge.start()

    signal.signal(signal.SIGINT, lambda *_: bridge.stop())
    signal.pause()


if __name__ == "__main__":
    main()

rerun_bridge = RerunBridgeModule.blueprint
