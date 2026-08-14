from pathlib import Path


def twin_dir() -> Path:
    """Return the twin repository root (one level above this package)."""
    return Path(__file__).parent.parent


def resolve(name: str) -> Path:
    """Resolve a path registered under the ``jubilee.paths`` entry point group.

    Raises RuntimeError if the entry point is not registered.
    """
    from importlib.metadata import entry_points
    eps = [ep for ep in entry_points(group="jubilee.paths") if ep.name == name]
    if not eps:
        raise RuntimeError(
            f"jubilee.paths/{name} entry point not found. "
            "Ensure science-jubilee is installed in this environment."
        )
    return Path(eps[0].load()())
