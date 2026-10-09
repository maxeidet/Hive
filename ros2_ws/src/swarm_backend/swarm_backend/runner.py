"""Helpers for writing flight sequences as plain top-to-bottom code.

The ROS executor spins in a background thread (so setpoint streams and state updates
keep running) while the main thread issues commands and waits on drone state.
"""
import threading
import time
from collections.abc import Callable
from contextlib import contextmanager

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

# PX4 only disarms once its land detector is sure, which takes a while after touchdown,
# and much longer in wall-clock time when Gazebo runs slower than real time.
LAND_TIMEOUT_S = 180.0


@contextmanager
def spinning(node: Node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield executor
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)


def wait_until(predicate: Callable[[], bool], timeout: float, what: str,
               poll_period: float = 0.1) -> None:
    """Block until predicate() is true, or raise TimeoutError naming `what`."""
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError(f'Timed out after {timeout:.0f} s waiting for {what}')
        time.sleep(poll_period)
