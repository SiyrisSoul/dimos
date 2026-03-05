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

from abc import ABC
from collections import defaultdict
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.docker_runner import is_docker_module
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleBase, ModuleSpec
from dimos.core.resource import Resource
from dimos.core.worker import WorkerPython
from dimos.core.worker_base import Worker
from dimos.core.worker_docker import WorkerDocker
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.resource_monitor.monitor import StatsMonitor
    from dimos.core.rpc_client import ModuleProxy
    from dimos.core.transport import PubSubTransport

logger = setup_logger()


class ModuleCoordinator(Resource):  # type: ignore[misc]
    """
    This module controls when and where modules are deployed.
    """

    _global_config: GlobalConfig
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[ModuleBase], ModuleProxy]
    _stats_monitor: StatsMonitor | None = None

    def __init__(
        self,
        n_workers: int | Callable[[int], int] | None = None,
        memory_limit: str | None = None,
        cfg: GlobalConfig = global_config,
    ) -> None:
        self._global_config = cfg
        self._n_workers: int | Callable[[int], int] = (
            n_workers if n_workers is not None else cfg.n_workers
        )
        self._memory_limit = memory_limit or cfg.memory_limit
        self._deployed_modules = {}
        self._closed = False
        self._workers: list[Worker] = []
        self._docker_workers: list[Worker] = []

    def start(self) -> None:
        if self._global_config.dtop:
            from dimos.core.resource_monitor.monitor import StatsMonitor

            self._stats_monitor = StatsMonitor(self)
            self._stats_monitor.start()

    @property
    def workers(self) -> list[Worker]:
        return list(self._workers)

    def _select_worker(self, module_class: type[ModuleBase]) -> Worker:
        """Pick the right worker for a module: docker worker or least-loaded process worker."""
        if is_docker_module(module_class):
            docker_worker = WorkerDocker()
            self._docker_workers.append(docker_worker)
            return docker_worker

        # Compute max workers allowed
        if callable(self._n_workers):
            max_workers = self._n_workers(len(self._deployed_modules))
        else:
            max_workers = self._n_workers
        assert max_workers > 0, "Must specify a positive number of workers"

        # Spawn a new worker on demand if under the limit
        if len(self._workers) < max_workers:
            worker = WorkerPython()
            worker.start_process()
            self._workers.append(worker)

        return min(self._workers, key=lambda w: w.module_count)

    def stop(self) -> None:
        if self._stats_monitor is not None:
            self._stats_monitor.stop()
            self._stats_monitor = None

        for module_class, module in reversed(self._deployed_modules.items()):
            logger.info("Stopping module...", module=module_class.__name__)
            try:
                module.stop()
            except Exception:
                logger.error("Error stopping module", module=module_class.__name__, exc_info=True)
            logger.info("Module stopped.", module=module_class.__name__)

        self._close_all()

    def _close_all(self) -> None:
        if self._closed:
            return
        self._closed = True

        logger.info("Shutting down all workers...")

        for worker in reversed(self._docker_workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down docker worker: {e}", exc_info=True)
        self._docker_workers.clear()

        for worker in reversed(self._workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down worker: {e}", exc_info=True)
        self._workers.clear()

        logger.info("All workers shut down")

    def deploy(
        self,
        module_class: type[ModuleBase[Any]],
        global_config: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> ModuleProxy:
        if self._closed:
            raise RuntimeError("ModuleCoordinator is closed")

        worker = self._select_worker(module_class)
        module = worker.deploy_module(module_class, global_config, kwargs=kwargs)
        self._deployed_modules[module_class] = module  # type: ignore[assignment]
        return module  # type: ignore[return-value]

    def deploy_parallel(self, module_specs: list[ModuleSpec]) -> list[ModuleProxy]:
        if self._closed:
            raise RuntimeError("ModuleCoordinator is closed")

        # Assign each module to a worker, reserving slots for load balancing
        assignments: list[tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]]] = []
        for module_class, global_config_val, kwargs in module_specs:
            worker = self._select_worker(module_class)
            worker.reserve_slot()
            assignments.append((worker, module_class, global_config_val, kwargs))

        # Deploy all modules concurrently
        def _deploy(
            item: tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]],
        ) -> Any:
            worker, module_class, gc, kwargs = item
            return worker.deploy_module(module_class, gc, kwargs=kwargs)

        with ThreadPoolExecutor(max_workers=max(len(assignments), 1)) as pool:
            results = list(pool.map(_deploy, assignments))

        for (module_class, _, _), module in zip(module_specs, results, strict=True):
            self._deployed_modules[module_class] = module  # type: ignore[assignment]
        return results  # type: ignore[return-value]

    def group_deploy(
        self,
        module_specs: list[ModuleSpec],
        stream_wiring: list[tuple[type[ModuleBase], str, PubSubTransport[Any]]],
        module_ref_wiring: list[tuple[type[ModuleBase], str, type[ModuleBase]]],
    ) -> None:
        """Orchestrate: deploy all modules, wire streams/rpc/refs, then start."""
        self.deploy_parallel(module_specs)
        self._wire_streams(stream_wiring)
        self._wire_rpc_methods()
        self._wire_module_refs(module_ref_wiring)
        self.start_all_modules()

    def _wire_streams(
        self,
        connections: list[tuple[type[ModuleBase], str, PubSubTransport[Any]]],
    ) -> None:
        for module_class, stream_name, transport in connections:
            instance = self.get_instance(module_class)
            instance.set_transport(stream_name, transport)  # type: ignore[union-attr]

    def _wire_rpc_methods(self) -> None:
        rpc_methods: dict[str, Any] = {}
        rpc_methods_dot: dict[str, Any] = {}

        # Track interface methods to detect ambiguity.
        interface_methods: defaultdict[str, list[tuple[type[ModuleBase], Callable[..., Any]]]] = (
            defaultdict(list)
        )
        interface_methods_dot: defaultdict[
            str, list[tuple[type[ModuleBase], Callable[..., Any]]]
        ] = defaultdict(list)

        for module_class, module_proxy in self._deployed_modules.items():
            for method_name in module_class.rpcs.keys():  # type: ignore[attr-defined]
                method_for_rpc_client = getattr(module_proxy, method_name)
                rpc_methods[f"{module_class.__name__}_{method_name}"] = method_for_rpc_client
                rpc_methods_dot[f"{module_class.__name__}.{method_name}"] = method_for_rpc_client

                for base in module_class.mro():
                    if (
                        base is not Module
                        and issubclass(base, ABC)
                        and hasattr(base, method_name)
                        and getattr(base, method_name, None) is not None
                    ):
                        interface_key = f"{base.__name__}.{method_name}"
                        interface_methods_dot[interface_key].append(
                            (module_class, method_for_rpc_client)
                        )
                        interface_key_underscore = f"{base.__name__}_{method_name}"
                        interface_methods[interface_key_underscore].append(
                            (module_class, method_for_rpc_client)
                        )

        # Add non-ambiguous interface methods
        for interface_key, implementations in interface_methods_dot.items():
            if len(implementations) == 1:
                rpc_methods_dot[interface_key] = implementations[0][1]
        for interface_key, implementations in interface_methods.items():
            if len(implementations) == 1:
                rpc_methods[interface_key] = implementations[0][1]

        # Fulfil method requests (so modules can call each other).
        for module_class, module_proxy in self._deployed_modules.items():
            for method_name in module_class.rpcs.keys():  # type: ignore[attr-defined]
                if not method_name.startswith("set_"):
                    continue

                linked_name = method_name.removeprefix("set_")
                self._check_rpc_ambiguity(linked_name, interface_methods, module_class)

                if linked_name not in rpc_methods:
                    continue

                getattr(module_proxy, method_name)(rpc_methods[linked_name])

            for requested_method_name in module_proxy.get_rpc_method_names():  # type: ignore[union-attr]
                self._check_rpc_ambiguity(
                    requested_method_name, interface_methods_dot, module_class
                )

                if requested_method_name not in rpc_methods_dot:
                    continue

                module_proxy.set_rpc_method(  # type: ignore[union-attr]
                    requested_method_name, rpc_methods_dot[requested_method_name]
                )

    def _wire_module_refs(
        self,
        connections: list[tuple[type[ModuleBase], str, type[ModuleBase]]],
    ) -> None:
        for base_module, ref_name, target_module in connections:
            base_module_proxy = self.get_instance(base_module)
            target_module_proxy = self.get_instance(target_module)
            setattr(base_module_proxy, ref_name, target_module_proxy)
            base_module_proxy.set_module_ref(ref_name, target_module_proxy)  # type: ignore[union-attr]

    @staticmethod
    def _check_rpc_ambiguity(
        requested_method_name: str,
        interface_methods: dict[str, list[tuple[type[ModuleBase], Callable[..., Any]]]]
        | defaultdict[str, list[tuple[type[ModuleBase], Callable[..., Any]]]],
        requesting_module: type[ModuleBase],
    ) -> None:
        if (
            requested_method_name in interface_methods
            and len(interface_methods[requested_method_name]) > 1
        ):
            modules_str = ", ".join(
                impl[0].__name__ for impl in interface_methods[requested_method_name]
            )
            raise ValueError(
                f"Ambiguous RPC method '{requested_method_name}' requested by "
                f"{requesting_module.__name__}. Multiple implementations found: "
                f"{modules_str}. Please use a concrete class name instead."
            )

    def start_all_modules(self) -> None:
        modules = list(self._deployed_modules.values())
        with ThreadPoolExecutor(max_workers=len(modules)) as executor:
            list(executor.map(lambda m: m.start(), modules))

        module_list = list(self._deployed_modules.values())
        for module in modules:
            if hasattr(module, "on_system_modules"):
                module.on_system_modules(module_list)

    def get_instance(self, module: type[ModuleBase]) -> ModuleProxy:
        return self._deployed_modules.get(module)  # type: ignore[return-value, no-any-return]

    def loop(self) -> None:
        stop = threading.Event()
        try:
            stop.wait()
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
