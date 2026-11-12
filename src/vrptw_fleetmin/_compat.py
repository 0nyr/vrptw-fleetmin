"""Check that the installed PyVRP is one this package can use."""

from __future__ import annotations

from importlib import metadata, util

SUPPORTED_PYVRP = ">=0.12.2,<0.13"


def unsupported_pyvrp_message(installed_version: str) -> str:
    """Return the message raised when the installed PyVRP cannot run the ILS."""
    return (
        f"vrptw-fleetmin needs PyVRP {SUPPORTED_PYVRP}, but PyVRP {installed_version} is installed. "
        "The ILS uses the pyvrp.repair module, which PyVRP no longer ships from version 0.13. "
        "This usually happens when another requirement pulls a newer PyVRP, for example the 'pyvrp' extra of "
        "mamut-routing-lib, which asks for PyVRP 0.13.4 or later. "
        "Install mamut-routing-lib without that extra, then restore a supported PyVRP with: "
        f'pip install "pyvrp{SUPPORTED_PYVRP}"'
    )


def require_supported_pyvrp() -> None:
    """Raise an ImportError with an explanation when PyVRP lacks the module the ILS needs.

    Nothing is raised when PyVRP is not installed at all. The import that follows
    this check then reports the missing package in the usual way.
    """
    if util.find_spec("pyvrp") is None:
        return
    if util.find_spec("pyvrp.repair") is None:
        try:
            installed = metadata.version("pyvrp")
        except metadata.PackageNotFoundError:
            installed = "(unknown version)"
        raise ImportError(unsupported_pyvrp_message(installed))
