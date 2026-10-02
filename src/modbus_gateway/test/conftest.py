import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture(autouse=True)
def preserve_launch_event_loop():
    # Direct gateway callbacks and asyncio.run clear the policy's current loop.
    # Restore the caller's loop for later Humble launch tests in one collection.
    policy = asyncio.get_event_loop_policy()
    try:
        previous = policy.get_event_loop()
    except RuntimeError:
        previous = None
    try:
        yield
    finally:
        policy.set_event_loop(previous)
