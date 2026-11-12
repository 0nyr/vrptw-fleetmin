"""Tests of the PyVRP compatibility check."""

from __future__ import annotations

import pytest

from vrptw_fleetmin import _compat


def test_supported_pyvrp_passes() -> None:
    """The check is silent with the PyVRP version the package is installed with."""
    _compat.require_supported_pyvrp()


def test_message_names_the_versions_and_the_fix() -> None:
    message = _compat.unsupported_pyvrp_message("0.13.4")
    assert "PyVRP 0.13.4 is installed" in message
    assert "pyvrp.repair" in message
    assert "mamut-routing-lib" in message
    assert 'pip install "pyvrp>=0.12.2,<0.13"' in message


def test_missing_repair_module_raises_import_error(monkeypatch: pytest.MonkeyPatch) -> None:
    """A PyVRP without pyvrp.repair, as from version 0.13, gives the explanatory error."""
    real_find_spec = _compat.util.find_spec

    def find_spec_without_repair(name: str, package: str | None = None):
        if name == "pyvrp.repair":
            return None
        return real_find_spec(name, package)

    monkeypatch.setattr(_compat.util, "find_spec", find_spec_without_repair)
    monkeypatch.setattr(_compat.metadata, "version", lambda name: "0.13.4")
    with pytest.raises(ImportError, match="PyVRP 0.13.4 is installed"):
        _compat.require_supported_pyvrp()
