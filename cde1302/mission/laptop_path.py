"""
Make the course's laptop/ modules (comms, protocol, camera_stream,
apriltag_host) importable without copying or modifying them.

    import laptop_path  # noqa: F401  (must come before the course imports)
    from comms import CommsNode
"""

import sys
from pathlib import Path

LAPTOP_DIR = Path(__file__).resolve().parent.parent / "laptop"

if str(LAPTOP_DIR) not in sys.path:
    sys.path.insert(0, str(LAPTOP_DIR))
