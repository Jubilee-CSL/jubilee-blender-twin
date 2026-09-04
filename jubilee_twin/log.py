"""Shared logging setup for jubilee_twin.

Call get_logger(__name__) in each module.
colorlog is used when installed; plain logging is the fallback.

Convention used across the package:
  log.info()    — progress steps ("Step 2/4: placing tools...")
  log.warning() — file paths being read or written (shown bold-yellow so they're easy to spot)
  log.error()   — failures

Records are also mirrored into the trace recap when science_jubilee is present.
"""
import logging

_configured = False


def _setup() -> None:
    global _configured
    if _configured:
        return
    _configured = True

    root = logging.getLogger("jubilee_twin")
    if root.handlers:
        return
    root.setLevel(logging.DEBUG)
    root.propagate = False

    from jubilee_twin.trace import capture_logger

    capture_logger("jubilee_twin")

    try:
        import colorlog
        handler = colorlog.StreamHandler()
        handler.setFormatter(
            colorlog.ColoredFormatter(
                "%(log_color)s%(levelname)-8s%(reset)s "
                "%(cyan)s%(name)s%(reset)s  %(message)s",
                log_colors={
                    "DEBUG":    "white",
                    "INFO":     "green,bold",
                    "WARNING":  "yellow,bold",
                    "ERROR":    "red,bold",
                    "CRITICAL": "red,bold,bg_white",
                },
            )
        )
    except ImportError:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("%(levelname)-8s %(name)s  %(message)s")
        )

    root.addHandler(handler)


def get_logger(name: str) -> logging.Logger:
    _setup()
    return logging.getLogger(name)
