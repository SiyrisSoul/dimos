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
"""Unified timestamped sensor storage and replay."""

from abc import ABC, abstractmethod
import bisect
from collections.abc import Iterator
import time
from typing import Generic, TypeVar

import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable, Disposable
from reactivex.observable import Observable
from reactivex.scheduler import TimeoutScheduler

from dimos.types.timestamped import Timestamped

T = TypeVar("T", bound=Timestamped)


class SensorStore(Generic[T], ABC):
    """Unified storage + replay for timestamped sensor data.

    Implement 4 abstract methods for your backend (in-memory, pickle, sqlite, etc.).
    All iteration, streaming, and seek logic comes free from the base class.
    """

    # === Abstract - implement for your backend ===

    @abstractmethod
    def _save(self, timestamp: float, data: T) -> None:
        """Save data at timestamp."""
        ...

    @abstractmethod
    def _load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp. Returns None if not found."""
        ...

    @abstractmethod
    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        """Lazy iteration of (timestamp, data) in range."""
        ...

    @abstractmethod
    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """Find closest timestamp. Backend can optimize (binary search, db index, etc.)."""
        ...

    def save(self, *data: T) -> None:
        """Save one or more timestamped data items using their .ts attribute."""
        for item in data:
            self._save(item.ts, item)

    def pipe_save(self, source: Observable[T]) -> Observable[T]:
        """Operator for use with .pipe() - saves each item and passes through.

        Usage:
            observable.pipe(store.pipe_save).subscribe(...)
        """

        def _save_and_return(data: T) -> T:
            self.save(data)
            return data

        return source.pipe(ops.map(_save_and_return))

    def consume_stream(self, observable: Observable[T]) -> rx.abc.DisposableBase:
        """Subscribe to an observable and save each item.

        Usage:
            disposable = store.consume_stream(observable)
        """
        return observable.subscribe(on_next=self.save)

    def load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp."""
        return self._load(timestamp)

    def find_closest(
        self,
        timestamp: float,
        tolerance: float | None = None,
    ) -> T | None:
        """Find data closest to the given absolute timestamp."""
        closest_ts = self._find_closest_timestamp(timestamp, tolerance)
        if closest_ts is None:
            return None
        return self._load(closest_ts)

    def find_closest_seek(
        self,
        relative_seconds: float,
        tolerance: float | None = None,
    ) -> T | None:
        """Find data closest to a time relative to the start."""
        first = self.first_timestamp()
        if first is None:
            return None
        return self.find_closest(first + relative_seconds, tolerance)

    def first_timestamp(self) -> float | None:
        """Get the first timestamp in the store."""
        for ts, _ in self._iter_items():
            return ts
        return None

    def first(self) -> T | None:
        """Get the first data item in the store."""
        for _, data in self._iter_items():
            return data
        return None

    def iterate(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[T]:
        """Iterate over data items with optional seek/duration."""
        first = self.first_timestamp()
        if first is None:
            return

        # Calculate start timestamp
        if from_timestamp is not None:
            start = from_timestamp
        elif seek is not None:
            start = first + seek
        else:
            start = None

        # Calculate end timestamp
        end = None
        if duration is not None:
            start_ts = start if start is not None else first
            end = start_ts + duration

        while True:
            for _, data in self._iter_items(start=start, end=end):
                yield data
            if not loop:
                break

    def iterate_realtime(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[T]:
        """Iterate data, sleeping to match original timing."""
        prev_ts: float | None = None
        for data in self.iterate(
            seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
        ):
            if prev_ts is not None:
                delay = (data.ts - prev_ts) / speed
                if delay > 0:
                    time.sleep(delay)
            prev_ts = data.ts
            yield data

    def stream(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T]:
        """Stream data as Observable with timing control.

        Uses scheduler-based timing with absolute time reference to prevent drift.
        """

        def subscribe(
            observer: rx.abc.ObserverBase[T],
            scheduler: rx.abc.SchedulerBase | None = None,
        ) -> rx.abc.DisposableBase:
            sched = scheduler or TimeoutScheduler()
            disp = CompositeDisposable()
            is_disposed = False

            iterator = self.iterate(
                seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
            )

            # Get first message
            try:
                first_data = next(iterator)
            except StopIteration:
                observer.on_completed()
                return Disposable()

            # Establish timing reference (absolute time prevents drift)
            start_local_time = time.time()
            start_replay_time = first_data.ts

            # Emit first sample immediately
            observer.on_next(first_data)

            # Pre-load next message
            try:
                next_message: T | None = next(iterator)
            except StopIteration:
                observer.on_completed()
                return disp

            def schedule_emission(data: T) -> None:
                nonlocal next_message, is_disposed

                if is_disposed:
                    return

                # Pre-load the following message while we have time
                try:
                    next_message = next(iterator)
                except StopIteration:
                    next_message = None

                # Calculate absolute emission time
                target_time = start_local_time + (data.ts - start_replay_time) / speed
                delay = max(0.0, target_time - time.time())

                def emit(
                    _scheduler: rx.abc.SchedulerBase, _state: object
                ) -> rx.abc.DisposableBase | None:
                    if is_disposed:
                        return None
                    observer.on_next(data)
                    if next_message is not None:
                        schedule_emission(next_message)
                    else:
                        observer.on_completed()
                    return None

                sched.schedule_relative(delay, emit)

            if next_message is not None:
                schedule_emission(next_message)

            def dispose() -> None:
                nonlocal is_disposed
                is_disposed = True
                disp.dispose()

            return Disposable(dispose)

        return rx.create(subscribe)


class InMemoryStore(SensorStore[T]):
    """In-memory storage using dict. Good for live use."""

    def __init__(self) -> None:
        self._data: dict[float, T] = {}
        self._sorted_timestamps: list[float] | None = None

    def _save(self, timestamp: float, data: T) -> None:
        self._data[timestamp] = data
        self._sorted_timestamps = None  # Invalidate cache

    def _load(self, timestamp: float) -> T | None:
        return self._data.get(timestamp)

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        for ts in self._get_sorted_timestamps():
            if start is not None and ts < start:
                continue
            if end is not None and ts >= end:
                break
            yield (ts, self._data[ts])

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        timestamps = self._get_sorted_timestamps()
        if not timestamps:
            return None

        pos = bisect.bisect_left(timestamps, timestamp)

        candidates = []
        if pos > 0:
            candidates.append(timestamps[pos - 1])
        if pos < len(timestamps):
            candidates.append(timestamps[pos])

        if not candidates:
            return None

        closest = min(candidates, key=lambda ts: abs(ts - timestamp))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _get_sorted_timestamps(self) -> list[float]:
        if self._sorted_timestamps is None:
            self._sorted_timestamps = sorted(self._data.keys())
        return self._sorted_timestamps
