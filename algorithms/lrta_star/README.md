# LRTA* (Learning Real-Time A*)

## One-sentence idea

You only have time to look at your **neighbors**: step to the best one, then bump the heuristic of the cell you left so you don’t loop forever.

## When to use / when not to

- Use when a full D* Lite repair will not fit in the frame and the agent must **move anyway** (Korf 1990).
- Do not use when you can afford a real search — first-arrival paths are long until `h` has been trained by walking.
- Fine as a FOW extra: unknown cells start with Euclidean `h`; walls teach `h` the long way around.

## World it assumes

Grid. Heuristic starts admissible (Euclidean / octile).

## How it works

From `s`:

1. For each neighbor `n`, look at `c(s,n) + h(n)`.
2. Step to the min.
3. Set `h(s) ←` that min (so `h` becomes a better underestimate of true remaining cost).
4. Repeat.

If you would loop in a U-trap, `h` in the pocket rises until leaving looks cheaper. That is the “learning.” Many agents can share the same `h` table (a cheap stigmergy).

## Invariants / guarantees

With an initially admissible `h`, LRTA* reaches the goal on finite graphs (with some caveats on ties). Not optimal on the first trial; `h` converges toward true distances after enough trips.

## Complexity

O(degree) per step. Memory: one `h` per cell.

## Relation to others

- Real-time sibling of A*.
- D* Lite is the “I can think for a bit” version; LRTA* is the “I must step now” version.

## Swarm / cache note

**Shared `h` table** is useful: the first unit that walks the U-trap raises `h` for everyone. That is the one place a pheromone-like map *does* help, and it is already `h`. We still don’t need a separate pheromone planner.

## Fog-of-war note

`h` updates only for visited cells. Unknown stays Euclidean (freespace). Last-seen walls are just blocked neighbors.

## Debug overlay contract

`h` heatmap (watch the pocket fill), current lookahead scores, path so far. Panel: steps, max `h` raise this second.

## Sources

- Korf, R. E. (1990). Real-time heuristic search. *Artificial Intelligence*.
