from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from swarmpathfinding.generators.packs import build_spec, pack_specs
from swarmpathfinding.worlds.grid import GridWorld


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Materialize pack maps under maps/generated/.")
    parser.add_argument("--pack", default="single_static")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="maps/generated")
    parser.add_argument("--limit", type=int, default=None)
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    specs = pack_specs(args.pack, seeds=(args.seed,))
    if args.limit is not None:
        specs = specs[: args.limit]
    n = 0
    for spec in specs:
        scenario = build_spec(spec)
        safe = spec.scenario_id.replace(".", "p")
        if isinstance(scenario.world, GridWorld):
            path = out / f"{safe}.npz"
            np.savez(
                path,
                blocked=scenario.world.blocked,
                width=scenario.world.width,
                height=scenario.world.height,
            )
        else:
            path = out / f"{safe}.txt"
            lines = [
                f"# {spec.scenario_id} occupancy={scenario.density:.4f}",
                f"{scenario.world.width} {scenario.world.height}",
            ]
            for obs in scenario.world.obstacles:
                verts = " ".join(f"{v.x:.3f},{v.y:.3f}" for v in obs.vertices())
                lines.append(
                    f"{obs.id} v={obs.velocity.x:.3f},{obs.velocity.y:.3f} {verts}"
                )
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        n += 1
    print(f"wrote {n} maps under {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
