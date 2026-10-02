# SPDX-License-Identifier: Apache-2.0
"""Make the source package importable before the ROS workspace is installed."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
