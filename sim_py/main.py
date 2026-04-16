from __future__ import annotations

from sim_py.pipeline import SimulationSystem


def main() -> None:
    system = SimulationSystem()
    snapshot = system.step()
    print(snapshot)


if __name__ == "__main__":
    main()
