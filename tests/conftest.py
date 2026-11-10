import sys
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from rik import config  # noqa: E402


@pytest.fixture
def cfgs():
    return {p.stem.split("_")[1]: config.load(p) for p in config.default_paths()}
