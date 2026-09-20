"""同一进程启动接收节点与发布节点，共用一个 LaDDS 传输客户端。"""

from __future__ import annotations

import sys
from typing import Any, Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .inbound_node import LaDDSInboundNode, _short_error
from .ladds_client import LaDDSClient
from .outbound_node import LaDDSOutboundNode


def main(
    args: Optional[list[str]] = None,
    *,
    native_client_factory: Callable[..., Any] = LaDDSClient,
    tcp_client_factory: Optional[Callable[..., Any]] = None,
) -> None:
    """Initialize ROS, spin inbound and outbound nodes, then release transport."""
    initialized = False
    inbound: Optional[LaDDSInboundNode] = None
    outbound: Optional[LaDDSOutboundNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        rclpy.init(args=args)
        initialized = True
        inbound = LaDDSInboundNode(
            native_client_factory=native_client_factory,
            tcp_client_factory=tcp_client_factory,
        )
        outbound = LaDDSOutboundNode(
            inbound.bridge_config, inbound.client, inbound.header_cache
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(inbound)
        executor.add_node(outbound)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logger = inbound.get_logger() if inbound is not None else None
        if logger is not None:
            logger.fatal(f"LaDDS bridge stopped: {_short_error(exc)}")
        else:
            print(f"LaDDS bridge failed to start: {exc}", file=sys.stderr)
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        try:
            if outbound is not None:
                outbound.destroy_node()
            if inbound is not None:
                inbound.close()
                inbound.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()
