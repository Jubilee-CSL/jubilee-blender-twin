"""Trace recap, provided by science_jubilee.

The twin runs standalone, so when science_jubilee is absent every call becomes a
no-op and no recap is written — the log still carries the same information.
"""

from __future__ import annotations

try:
    from science_jubilee.trace import (  # noqa: F401
        FAILED,
        OK,
        PARTIAL,
        SKIPPED,
        Section,
        Trace,
        capture_logger,
        clear_records,
        flush,
        get_records,
        session,
        start_session,
    )

    AVAILABLE = True

except ImportError:  # pragma: no cover - exercised only in standalone installs
    AVAILABLE = False

    FAILED, OK, SKIPPED, PARTIAL = "failed", "ok", "skipped", "partial"

    class Section:  # type: ignore[no-redef]
        def __init__(self, name: str = "") -> None:
            self.name = name
            self.steps: list = []

        def step(self, label, status, detail="") -> None: ...
        def failed(self, label, detail="") -> None: ...
        def ok(self, label, detail="") -> None: ...
        def skipped(self, label, detail="") -> None: ...
        def partial(self, label, detail="") -> None: ...

    class Trace:  # type: ignore[no-redef]
        def __init__(self, title: str = "") -> None:
            self.title = title
            self.sections: list = []

        def section(self, name: str, reset: bool = False) -> Section:
            return Section(name)

        def result(self, label, text="", path="") -> None: ...
        def failed(self, label, detail="") -> None: ...
        def ok(self, label, detail="") -> None: ...
        def skipped(self, label, detail="") -> None: ...
        def partial(self, label, detail="") -> None: ...

    _session = Trace()

    def start_session(out_dir=None, title: str = "") -> Trace:
        return _session

    def session(out_dir=None, title: str = "") -> Trace:
        return _session

    def flush(out_dir=None, name=None):
        return None

    def capture_logger(name: str) -> None: ...

    def get_records() -> list:
        return []

    def clear_records() -> None: ...
