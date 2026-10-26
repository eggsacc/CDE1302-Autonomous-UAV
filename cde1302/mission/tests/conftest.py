"""Make mission/ and the course's laptop/ importable from the tests."""

import sys
from pathlib import Path

import pytest

MISSION_DIR = Path(__file__).resolve().parents[1]
LAPTOP_DIR = MISSION_DIR.parent / "laptop"
TMP_DIR = Path(__file__).resolve().parent / ".pytest_tmp"

for p in (MISSION_DIR, LAPTOP_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    """Keep tmp_path folders in mission/tests/.pytest_tmp/ (gitignored).

    pytest's default, %TEMP%\\pytest-of-<user>, breaks on Windows when that
    folder was once created with other permissions ("Access is denied").
    pytest empties this folder at the start of every run.  Pass --basetemp
    to override.
    """
    if not config.option.basetemp:
        config.option.basetemp = str(TMP_DIR)
