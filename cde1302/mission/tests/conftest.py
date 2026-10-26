"""Make mission/ and the course's laptop/ importable from the tests."""

import sys
from pathlib import Path

MISSION_DIR = Path(__file__).resolve().parents[1]
LAPTOP_DIR = MISSION_DIR.parent / "laptop"

for p in (MISSION_DIR, LAPTOP_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
