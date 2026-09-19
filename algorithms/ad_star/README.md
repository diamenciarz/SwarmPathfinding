# AD* (Anytime Dynamic A*)

## One-sentence idea

ARA* meets D* Lite: while time remains, shrink `w` and improve the path; when the map (or last-seen map) changes, **repair** instead of throwing the search away.

## When to use / when not to

- Use when you need **both** a frame budget **and** a map that twitches (fog of war, a wall deleted, a door). This is the “I have 5 ms this frame” algorithm for robots (Likhachev, Ferguson, Gordon, Stentz, Thrun 2005).
- Do not use on a static known map — ARA* or weighted A* is enough.
- Do not use if you always want a true optimum and can wait — D* Lite with `w = 1`.

## World it assumes

Same graph as D* Lite (goal-rooted grid). Heuristic admissible; weights as in ARA*.

## How it works

Keep D* Lite’s `g` / `rhs` / queue / `km`. Search with inflated `f` like weighted A* / ARA*.

- **Time left, map quiet:** decrease `w`, reuse inconsistent states (`INCONS`), improve the path, tighten the bound.
- **Edge costs changed:** the same D* Lite update (`rhs` of touched vertices, queue) but under the *current* `w`. If the change is big and the current path looks badly wrong, you may **raise** `w` again to get a feasible path quickly, then tighten.

You always have some path to follow (after the first solution) with a published bound.

We implement AD* **after** D* Lite (`km`) and ARA* are correct. It is not a different first version of D*.

## Invariants / guarantees

Anytime and incremental. Bounded-suboptimal at the current `w`. Optimal if you let `w` fall to 1 and the queue drain.

## Complexity

Repairs scale with the ripple, like D* Lite; extra work from multiple `w` passes, like ARA*.

## Relation to others

- Parents: [ARA*](../ara_star/README.md), [D\* Lite](../d_star_lite/README.md).
- Continuous cousin: anytime RRT* / BIT* + RRTX.

## Swarm / cache note

One AD* field per goal is possible but heavier than a plain D* Lite field. Prefer D* Lite + flow tiles unless you truly need the anytime knob on a changing map.

## Fog-of-war note

Excellent fit: last-seen twitches as scouts look, and you only have a slice of the frame. Run AD* on the last-seen grid.

## Debug overlay contract

All of D* Lite’s overlays plus current `w`, bound, and a cost-vs-time strip. Flash whether the last tick was a *tighten* or a *repair*.

## Sources

- Likhachev, Ferguson, Gordon, Stentz, Thrun (2005). Anytime Dynamic A*. *ICAPS*.
- Same authors (2008). Anytime search in dynamic graphs. *Artificial Intelligence* 172.
