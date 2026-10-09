# SPDX-License-Identifier: Apache-2.0
"""Reserve a ROS domain independently of caller environment and other runs."""

import fcntl
import json
import os
from pathlib import Path
import random


def domain_choices():
    """Return only non-inherited domains with a safe complete RTPS envelope."""
    try:
        inherited = int(os.environ.get("ROS_DOMAIN_ID", "-1"))
    except ValueError:
        inherited = -1
    ephemeral_start = int(
        Path("/proc/sys/net/ipv4/ip_local_port_range").read_text().split()[0]
    )
    choices = [
        domain
        for domain in range(20, 70)
        if domain != inherited and 7400 + 250 * domain + 249 < ephemeral_start
    ]
    random.SystemRandom().shuffle(choices)
    return choices


def lease_metadata_available(handle, domain):
    """Respect V1 quarantine; unknown or mismatched metadata fails closed."""
    handle.seek(0)
    content = handle.read()
    if not content.strip():
        return True
    try:
        metadata = json.loads(content)
    except ValueError:
        return False
    return (
        isinstance(metadata, dict)
        and type(metadata.get("domain_id")) is int
        and metadata["domain_id"] == domain
        and metadata.get("quarantined") is False
    )


class Domain:
    def __init__(self):
        self.lock = None
        # Repository fixtures use domains 72 and above. This separate pool also
        # keeps RTPS discovery/unicast ports below Linux's ephemeral UDP range:
        # 7400 + 250 * domain + participant offset must stay below the host's
        # configured ephemeral range, including its full 250-port envelope.
        for domain in domain_choices():
            handle = Path(f"/tmp/factory-fleet-domain-{domain}.lock").open("a+")
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                continue
            if not lease_metadata_available(handle, domain):
                handle.close()
                continue
            self.lock, self.id = handle, domain
            return
        raise RuntimeError("no isolated fleet ROS domain available")

    def close(self):
        if self.lock:
            self.lock.close()
            self.lock = None
