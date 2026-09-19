# BIT* (Batch Informed Trees)

## One-sentence idea

Sample a **batch** of points, treat them as an implicit graph, run an A*-like search on that graph, then sample a denser batch and **reuse** — anytime continuous planning that usually beats RRT* to a good path.

## When to use / when not to

- Use as the **default anytime planner** on static (or slowly changing) continuous maps.
- Do not use if you only need a crude feasible path in a millisecond — RRT-Connect may still win the first-path race on some maps.
- Descendants **AIT\*** / **EIT\*** add better heuristics; mention them, implement BIT* first.

## World it assumes

Continuous 2D, same collision as RRT*.

## How it works (intuition, not the full paper)

RRT* builds a tree one sample at a time, locally. BIT* says: a set of samples *is* a random geometric graph (connect points closer than r). Search that graph in **best-first** order, like A* / LPA*, using a heuristic (Euclidean to goal).

1. Sample a batch in the informed ellipse (once a path exists; the whole space before that).
2. Consider edges in order of estimated quality (`g + h` of the edge).
3. Collision-check lazily: only when the edge is a candidate for improving something.
4. When the batch is exhausted, add more samples, denser r, reuse previous `g` values (the LPA* flavor).

You get a feasible path early (coarse batch) and a shorter path later (denser batches), with less wasted collision checking than RRT*.

## Invariants / guarantees

Probabilistically complete. Asymptotically optimal. Anytime: current best path is always available.

## Complexity

Dominated by collision checks, like all samplers. BIT* typically spends them on edges that could actually improve the solution.

## Relation to others

- Mixes sampling (RRT*/PRM*) with incremental graph search (LPA*).
- Informed RRT* is the ellipse idea without batches.
- RRTX is the *dynamic* sampler; BIT* is the *static anytime* sampler.

## Swarm / cache note

Samples + current graph can be shared as a roadmap for one map. Different starts still need their own search, unless you keep a goal-rooted tree on the same samples (possible, similar spirit to D* Lite on a sampled graph).

## Fog-of-war note

Batches live in last-seen free + unknown. A newly seen wall invalidates edges; you can drop them and continue (poor man’s RRTX) or restart the batch.

## Debug overlay contract

Samples (current batch vs old), edges considered, current best path, informed ellipse, lazy edges not yet checked (dashed). Panel: batch index, best cost vs time, collision checks.

## Sources

- Gammell, Srinivasa, Barfoot. Batch Informed Trees (BIT*). *IEEE ICRA / RA-L* lineage.
- AIT*, EIT* follow-up papers (Gammell et al.).
