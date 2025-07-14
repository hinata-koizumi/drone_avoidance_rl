import importlib


def test_manual_control_import():
    assert importlib.import_module("manual_control") is not None


def test_simulator_import():
    sim_module = importlib.import_module("manual_control.simple_simulator")
    assert hasattr(sim_module, "main") 