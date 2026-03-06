"""
Ableware Simulation — entry point.

Modes:
  demo         Automated scenario demo (auto-cycles through weight scenarios)
  interactive  Manual control via keyboard (j/k weight, n/m target, u/i force, q quit)
  serve        Hub-driven mode: opens HTTP API on --port so the Ableware hub
               can send UP/DOWN/START/STOP commands from the Pi or dashboard
  serial       Serial bridge (JSON-over-serial from hardware)
  quit         Exit

Usage:
  python3 lift_actuator_sim/main.py [mode] [--port PORT]
"""

import sys
import threading
from http.server import HTTPServer

from wheelchair_sim_3d import (
    WheelchairLiftSim3D,
    _SimAPIHandler,
    _sim_ref as _sim_module,
)
import wheelchair_sim_3d as _sim_module_ref
from core.sim_constants import ACT_STROKE
from UI.Terminal_UI import prompt_mode, show_mode_screen


def _run_serve(port: int = 8001) -> None:
    """Start the HTTP API and run the interactive loop with hub control."""
    sim = WheelchairLiftSim3D()
    _sim_module_ref._sim_ref = sim

    api_server = HTTPServer(('', port), _SimAPIHandler)
    api_thread = threading.Thread(target=api_server.serve_forever, daemon=True)
    api_thread.start()

    print(f"[Ableware] API server listening on :{port}")
    print("[Ableware] Hub controls lift target. Keyboard still adjusts weight/force/speed.")
    print("[Ableware] Press q in the PyBullet window terminal to quit.")

    sim.run(external_control=True)


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="Ableware Wheelchair Sling Lift Simulation")
    ap.add_argument("mode", nargs="?", default=None,
                    choices=["demo", "interactive", "serve", "serial", "quit"],
                    help="Run mode (omit to get interactive prompt)")
    ap.add_argument("--port", type=int, default=8001,
                    help="HTTP API port for serve mode (default: 8001)")
    args = ap.parse_args()

    arg_mode = args.mode

    while True:
        mode = arg_mode if arg_mode is not None else prompt_mode(default="demo")

        if arg_mode is None:
            show_mode_screen(mode)

        if mode == "demo":
            try:
                sim = WheelchairLiftSim3D()
                sim.run_demo()
            except KeyboardInterrupt:
                pass
            if arg_mode is not None:
                break
            continue

        if mode == "interactive":
            try:
                sim = WheelchairLiftSim3D()
                sim.run()
            except KeyboardInterrupt:
                pass
            if arg_mode is not None:
                break
            continue

        if mode == "serve":
            try:
                _run_serve(port=args.port)
            except KeyboardInterrupt:
                pass
            break  # serve is always a one-shot run

        if mode == "serial":
            from serial_bridge import SerialSimulationBridge

            bridge = SerialSimulationBridge(
                sim_factory=WheelchairLiftSim3D,
                stroke_length=ACT_STROKE,
                serial_port="COM3",
                baudrate=115200,
                dt=1.0 / 240.0,
                headless=False,
            )
            try:
                bridge.start()
            except KeyboardInterrupt:
                pass
            finally:
                bridge.stop()
            if arg_mode is not None:
                break
            continue

        if mode == "quit":
            break

        print("Unknown mode. Choose: demo | interactive | serve | serial | quit")
        if arg_mode is not None:
            break


if __name__ == "__main__":
    main()
