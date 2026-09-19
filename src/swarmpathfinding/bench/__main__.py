from __future__ import annotations

import argparse
from pathlib import Path

from swarmpathfinding.bench.harness import run_pack, write_csv
from swarmpathfinding.generators.packs import PACKS, list_packs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a SwarmPathfinding scenario pack to CSV.")
    parser.add_argument("--pack", default="single_static", choices=list(PACKS) + ["all"])
    parser.add_argument("--algo", default="straight_line")
    parser.add_argument("--seeds", default="0", help="comma-separated ints")
    parser.add_argument("--out", default="", help="CSV path; default benches/out/<pack>.csv")
    parser.add_argument("--limit", type=int, default=None, help="cap specs (debug)")
    parser.add_argument("--list", action="store_true", help="print pack names and exit")
    args = parser.parse_args(argv)
    if args.list:
        print("\n".join(list_packs()))
        return 0
    seeds = tuple(int(s) for s in args.seeds.split(",") if s.strip() != "")
    packs = list(PACKS) if args.pack == "all" else [args.pack]
    all_rows = []
    for pack in packs:
        rows = run_pack(pack, args.algo, seeds=seeds, limit=args.limit)
        all_rows.extend(rows)
        print(f"{pack}: {len(rows)} rows")
    out = Path(args.out) if args.out else Path("benches") / "out" / f"{args.pack}.csv"
    write_csv(out, all_rows)
    print(f"wrote {out} ({len(all_rows)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
