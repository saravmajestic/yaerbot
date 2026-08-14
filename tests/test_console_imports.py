"""Import the console app with stubbed board modules.

Motivation: `ast.parse` proves a file is syntactically valid, which is NOT enough.
A module-level reference to a name defined further down the file parses fine and
then raises NameError at import — that shipped once (_DRIP_SPEED_MPS used CAL
before CAL existed) and took the container down on the robot, in the field.
This executes the module for real, so ordering bugs fail here instead of there.
"""
import importlib.util
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN = os.path.join(ROOT, "console", "python", "main.py")


def _stub_board_modules():
    """Minimal stand-ins for the App Lab runtime, which only exists on the board."""
    calls = []

    class Bridge:
        @staticmethod
        def call(name, *args):
            calls.append((name, *args))
            if name == "getBattery":
                return '{"volts":12.0,"pct":80}'
            if name == "getDiag":
                return '{"up_ms":1,"cmd":{"n":0},"move":{},"pins":{},"stops":0,' \
                       '"batt":{"raw":9900.0,"volts":12.0}}'
            return None

    class App:
        @staticmethod
        def run():                      # must NOT block the test run
            pass

    class WebUI:
        def __init__(self):
            self.handlers = {}

        def on_message(self, name, fn):
            self.handlers[name] = fn

        def send_message(self, name, payload=None):
            pass

    app_utils = types.ModuleType("arduino.app_utils")
    app_utils.Bridge, app_utils.App = Bridge, App
    web_ui = types.ModuleType("arduino.app_bricks.web_ui")
    web_ui.WebUI = WebUI
    arduino = types.ModuleType("arduino")
    bricks = types.ModuleType("arduino.app_bricks")

    sys.modules.update({"arduino": arduino, "arduino.app_utils": app_utils,
                        "arduino.app_bricks": bricks,
                        "arduino.app_bricks.web_ui": web_ui})
    return calls


def _load_main():
    _stub_board_modules()
    spec = importlib.util.spec_from_file_location("console_main", MAIN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # <- NameError / ordering bugs surface here
    return mod


def test_console_imports_without_error():
    mod = _load_main()
    assert mod.CAL["speed"] > 0
    # the bug this test exists for: derived at module level from CAL
    assert mod._DRIP_SPEED_MPS > 0
    assert mod._DRIP_SPEED_MPS < mod.CAL["speed"]   # follow duty is below drive duty


def test_every_registered_handler_exists_and_is_callable():
    """Catches a typo'd handler name in the ui.on_message block."""
    mod = _load_main()
    assert mod.ui.handlers, "no handlers registered"
    for name, fn in mod.ui.handlers.items():
        assert callable(fn), name
    for required in ("run_start", "run_stop", "run_pause", "plot_config",
                     "drip_config", "plot_mark", "get_plot", "get_diag"):
        assert required in mod.ui.handlers, f"{required} not registered"


def test_drip_defaults_give_a_four_seed_cross():
    mod = _load_main()
    assert mod._drip["angles"] == [0, 90]
    assert mod._drip_seeds_per_emitter() == 4
    assert mod._drip_rotates() is True


def test_handlers_run_against_the_stub_bridge():
    """Exercise the config/mark handlers so a bad key or type fails here."""
    mod = _load_main()
    h = mod.ui.handlers
    h["plot_config"](None, {"w": 3.0, "l": 4.0, "row_gap": 0.5,
                            "seed_gap": 0.25, "seeds_per_spot": 2, "dry": True})
    assert mod._plot["w"] == 3.0 and mod._plot["seed_gap"] == 0.25
    assert mod._run["dry"] is True

    h["drip_config"](None, {"emitter_gap": 0.5, "angles": [90, 0, 180, 90]})
    # 180 % 180 == 0, duplicates collapse, sorted
    assert mod._drip["angles"] == [0, 90]
    assert mod._drip["emitter_gap"] == 0.5

    h["drip_config"](None, {"angles": []})
    assert mod._drip["angles"] == [0], "must never end up with no arm position"

    for c in (1, 2, 3, 4):
        h["plot_mark"](None, {"corner": c})
    assert len(mod._plot["corners"]) == 4
    h["plot_clear"](None, {})
    assert mod._plot["corners"] == []


def test_start_refuses_without_marked_corners():
    mod = _load_main()
    mod._plot["corners"] = []
    mod.ui.handlers["run_start"](None, {"mode": "plain", "dry": True})
    assert mod._run["state"] == "idle"
    assert "corners" in mod._run["msg"]
