# Weighted A*

## One-sentence idea

A* that is allowed to be a little wrong: `f = g + w·h` with `w > 1` finds a path at most `w` times longer than optimal, usually with far fewer expansions.

## When to use / when not to

- Use when a **good-enough** path this frame beats a perfect path next frame. Default live-lab knob.
- Use `w ≈ 1.1` as a civilized default; `w = 2` is already quite greedy.
- Do not use when you need a proof of true optimality (`w = 1` is then just A*).

## World it assumes

Same as A*. `h` should still be **admissible**. The weight is what makes the *combination* inadmissible as an `f`, in a controlled way.

## How it works

Identical to A* except the heap key is `g + w·h`.

If `h` is admissible, the first time you expand the goal the path cost satisfies

`cost ≤ w · cost_optimal`.

That is **bounded-suboptimal**. Intuition: you are willing to expand a node that is `w` times more “hopefully close to the goal” than “already cheap.” You bias toward the goal, so you expand a narrower cone.

**Tiny picture:** on an open grid, A* (`w = 1`) fills a fat ellipse; `w = 2` fills a thin sausage along the start–goal line.

[ARA*](../ara_star/README.md) runs this several times, shrinking `w`, reusing expansions.

## Invariants / guarantees

- Complete (same as A*).
- Bounded-suboptimal with factor `w` when `h` is admissible.
- Not incremental by itself.

## Complexity

Expansions drop quickly as `w` grows, then flatten. Measure it; don’t guess.

## Relation to others

- Parent: A*.
- Child: ARA*, AD*.
- Contrast: greedy best-first is “`w = ∞`” in spirit.

## Swarm / cache note

Still per query. Combine with a shared field if N is large; use weight only for the *portal* A* or the one centroid path.

## Fog-of-war note

Fine on last-seen; still restart on edits unless you graduate to AD*.

## Debug overlay contract

Same as A*, plus a `w` slider and a live `cost / lower_bound` ratio. Heatmap of `f`. Panel: expansions vs a `w = 1` run if we keep one in the background for the demo.

## Sources

- Pohl (1970) / weighted heuristic search.
- Likhachev, Gordon, Thrun (2003) ARA* paper — § on why inflation works.
