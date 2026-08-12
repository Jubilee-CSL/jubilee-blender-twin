from pathlib import Path


def twin_dir() -> Path:
    """Return the twin repository root (one level above this package)."""
    return Path(__file__).parent.parent
