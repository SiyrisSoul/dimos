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

"""
Run the p1 + p2 behavior using Dask with separate processes for color and lidar logging.

- One Dask task mirrors p1.py (rerun init/grpc + dashboard server).
- One Dask task replays color_image.yaml.
- One Dask task replays lidar.yaml.

This keeps logging streams in distinct Dask worker processes (no threads).
"""

import dataclasses
import os
from pathlib import Path
import sys
import time

from dask.distributed import Client, LocalCluster
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
import yaml

from dimos.dashboard.server import start_dashboard_server_thread

default_rerun_grpc_port = 9876


@dataclasses.dataclass
class RerunInfo:
    logging_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    grpc_port: int = int(os.environ.get("RERUN_GRPC_PORT", default_rerun_grpc_port))
    server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%")
    url: str = os.environ.get(
        "RERUN_URL",
        f"rerun+http://localhost:{os.environ.get('RERUN_GRPC_PORT', default_rerun_grpc_port)!s}/proxy",
    )


rerun_info = RerunInfo()


class RerunConnection:
    def __init__(self) -> None:
        self.stream = rr.RecordingStream(rerun_info.logging_id, recording_id=rerun_info.logging_id)
        self.stream.connect_grpc(rerun_info.url)

    def log(self, msg: str, value, **kwargs) -> None:
        self.stream.log(msg, value, **kwargs)


def dashboard_process() -> None:
    """Mirrors p1.py: init rerun, send blueprint, start grpc and dashboard server."""

    print("[Dask Dashboard] calling rr.init")
    rr.init(rerun_info.logging_id, spawn=False, recording_id=rerun_info.logging_id)
    default_blueprint = rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(
                name="Spatial3D",
                origin="/",
                line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
            ),
        )
    )
    print("[Dask Dashboard] sending empty blueprint")
    rr.send_blueprint(default_blueprint)
    if not os.environ.get("RERUN_URL", None):
        print("[Dask Dashboard] starting rerun grpc")
        rr.serve_grpc(
            grpc_port=rerun_info.grpc_port,
            default_blueprint=default_blueprint,
            server_memory_limit=rerun_info.server_memory_limit,
        )
    thread = start_dashboard_server_thread(
        **{
            "auto_open": True,
            "terminal_commands": {"agent-spy": "htop", "lcm-spy": "dimos lcmspy"},
            "worker": 1,
        },
        rrd_url=rerun_info.url,
        keep_alive=True,
    )
    try:
        # Keep the dashboard thread alive; this process will be terminated by the scheduler/cluster.
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("[Dask Dashboard] interrupted, shutting down")
    finally:
        thread.join(timeout=1)


def log_stream_process(name: str, path_str: str) -> None:
    print(f"log process {name} started")
    rc = RerunConnection()
    path = Path(path_str)
    if not path.exists():
        print(f"[Dask Logging {name}] file {path} does not exist", file=sys.stderr)
        return

    with path.open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f):
            if not line.strip():
                continue
            try:
                parsed = yaml.unsafe_load(line) or []
            except Exception as error:
                print(f"[Dask Logging {name}] warning line:{line_number}: {error}")
                continue

            try:
                payload = parsed[0] if isinstance(parsed, list) else parsed
                print(f"[Dask Logging {name}] logging message {line_number}")
                rc.log(f"/{name}", payload.to_rerun(), strict=True)
            except Exception as error:
                print(f"[Dask Logging {name}] error: {error}")


def main() -> None:
    # At least 3 workers so dashboard + color + lidar each get their own process.
    cluster = LocalCluster(
        n_workers=1,
        threads_per_worker=4,
        processes=True,
        dashboard_address=None,
    )
    client = Client(cluster)

    dashboard_future = client.submit(dashboard_process, pure=False)
    # Give the dashboard/grpc server a moment to spin up before logging.
    time.sleep(1.5)
    print("starting color_future")  # <- this prints before "dashboard_process" starts
    color_future = client.submit(
        log_stream_process, "color_image", "./dimos/dashboard/support/color_image.yaml", pure=False
    )
    lidar_future = client.submit(
        log_stream_process, "lidar", "./dimos/dashboard/support/lidar.yaml", pure=False
    )

    # Wait for logging to finish.
    color_future.result()
    lidar_future.result()

    # Dashboard process will be torn down when the cluster closes.
    dashboard_future.cancel()

    client.close()
    cluster.close()


if __name__ == "__main__":
    main()
