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

from typing import TYPE_CHECKING
from unittest.mock import patch

import pytest

from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModuleConfig, is_docker_module
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.core.worker_docker import DockerRPCClient, WorkerDocker
from dimos.msgs.geometry_msgs import Vector3

if TYPE_CHECKING:
    from dimos.core.resource_monitor.stats import WorkerStats


class SimpleModule(Module):
    output: Out[Vector3]
    input: In[Vector3]

    counter: int = 0

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def increment(self) -> int:
        self.counter += 1
        return self.counter

    @rpc
    def get_counter(self) -> int:
        return self.counter


class AnotherModule(Module):
    value: int = 100

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def add(self, n: int) -> int:
        self.value += n
        return self.value

    @rpc
    def get_value(self) -> int:
        return self.value


class ThirdModule(Module):
    multiplier: int = 1

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def multiply(self, n: int) -> int:
        self.multiplier *= n
        return self.multiplier

    @rpc
    def get_multiplier(self) -> int:
        return self.multiplier


@pytest.fixture
def create_coordinator():
    dimos = None

    def _create():
        nonlocal dimos
        dimos = ModuleCoordinator()
        dimos.start()
        return dimos

    yield _create

    if dimos is not None:
        dimos.stop()


@pytest.mark.slow
def test_worker_manager_basic(create_coordinator):
    dimos = create_coordinator()
    module = dimos.deploy(SimpleModule, global_config)
    module.start()

    result = module.increment()
    assert result == 1

    result = module.increment()
    assert result == 2

    result = module.get_counter()
    assert result == 2

    module.stop()


@pytest.mark.slow
def test_worker_manager_multiple_different_modules(create_coordinator):
    dimos = create_coordinator()
    module1 = dimos.deploy(SimpleModule, global_config)
    module2 = dimos.deploy(AnotherModule, global_config)

    module1.start()
    module2.start()

    # Each module has its own state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Stop modules to clean up threads
    module1.stop()
    module2.stop()


@pytest.mark.slow
def test_worker_manager_parallel_deployment(create_coordinator):
    dimos = create_coordinator()
    modules = dimos.deploy_parallel(
        [
            (SimpleModule, (), {}),
            (AnotherModule, (), {}),
            (ThirdModule, (), {}),
        ]
    )

    assert len(modules) == 3
    module1, module2, module3 = modules

    # Start all modules
    module1.start()
    module2.start()
    module3.start()

    # Each module has its own state
    module1.increment()
    module2.add(50)
    module3.multiply(5)

    assert module1.get_counter() == 1
    assert module2.get_value() == 150
    assert module3.get_multiplier() == 5

    # Stop modules
    module1.stop()
    module2.stop()
    module3.stop()


@pytest.mark.slow
def test_collect_stats(create_coordinator):
    from dimos.core.resource_monitor.monitor import StatsMonitor

    dimos = create_coordinator()
    module1 = dimos.deploy(SimpleModule, global_config)
    module2 = dimos.deploy(AnotherModule, global_config)
    module1.start()
    module2.start()

    # Use a capturing logger to collect stats via StatsMonitor
    captured: list[list[WorkerStats]] = []

    class CapturingLogger:
        def log_stats(self, coordinator, workers):
            captured.append(workers)

    monitor = StatsMonitor(dimos, resource_logger=CapturingLogger(), interval=0.5)
    monitor.start()
    import time

    time.sleep(1.5)
    monitor.stop()

    assert len(captured) >= 1
    stats = captured[-1]
    assert len(stats) == 2

    for s in stats:
        assert s.alive is True
        assert s.pid > 0
        assert s.pss >= 0
        assert s.num_threads >= 1
        assert s.num_fds >= 0
        assert s.io_read_bytes >= 0
        assert s.io_write_bytes >= 0

    # At least one worker should report module names
    all_modules = [name for s in stats for name in s.modules]
    assert "SimpleModule" in all_modules
    assert "AnotherModule" in all_modules

    module1.stop()
    module2.stop()


@pytest.mark.slow
def test_worker_pool_modules_share_workers(create_coordinator):
    dimos = create_coordinator()
    module1 = dimos.deploy(SimpleModule, global_config)
    module2 = dimos.deploy(AnotherModule, global_config)

    module1.start()
    module2.start()

    # Verify isolated state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Verify modules are distributed across workers (default is 2 workers)
    assert len(dimos._workers) == 2
    total_modules = sum(w.module_count for w in dimos._workers)
    assert total_modules == 2

    module1.stop()
    module2.stop()


# ---------------------------------------------------------------------------
# Docker module deployment smoke tests (no Docker required)
# ---------------------------------------------------------------------------


class FakeDockerModuleConfig(DockerModuleConfig):
    docker_image: str = "fake:latest"


class FakeDockerModule(Module):
    default_config = FakeDockerModuleConfig

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def do_something(self) -> str:
        return "ok"


def test_is_docker_module_detection():
    """Modules with DockerModuleConfig-based default_config are detected."""
    assert is_docker_module(FakeDockerModule) is True
    assert is_docker_module(SimpleModule) is False


def test_docker_module_routes_to_docker_worker():
    """deploy() routes Docker modules through WorkerDocker, not regular Workers."""
    dimos = ModuleCoordinator()
    try:
        with patch.object(WorkerDocker, "deploy_module") as mock_deploy:
            from unittest.mock import MagicMock

            fake_dm = MagicMock()
            fake_dm.rpc = MagicMock()
            fake_dm._unsub_fns = []
            fake_client = DockerRPCClient(fake_dm, FakeDockerModule)
            mock_deploy.return_value = fake_client

            result = dimos.deploy(FakeDockerModule, global_config)

            mock_deploy.assert_called_once_with(FakeDockerModule, global_config, kwargs={})
            assert isinstance(result, DockerRPCClient)
            assert isinstance(result, RPCClient)
    finally:
        dimos.stop()


def test_docker_module_not_in_deploy_parallel_regular_path():
    """deploy_parallel() separates Docker modules from regular ones."""
    dimos = ModuleCoordinator()
    try:
        with patch.object(WorkerDocker, "deploy_module") as mock_docker:
            from unittest.mock import MagicMock

            fake_dm = MagicMock()
            fake_dm.rpc = MagicMock()
            fake_dm._unsub_fns = []
            fake_client = DockerRPCClient(fake_dm, FakeDockerModule)
            mock_docker.return_value = fake_client

            results = dimos.deploy_parallel(
                [
                    (FakeDockerModule, global_config, {}),
                ]
            )

            assert len(results) == 1
            mock_docker.assert_called_once()
            assert isinstance(results[0], DockerRPCClient)
    finally:
        dimos.stop()
