# References

Papers and notes cited in the algorithm READMEs and in [01-sota-overview.md](01-sota-overview.md). Not a complete bibliography of pathfinding.

## Grid / heuristic / incremental

- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Trans. Systems Science and Cybernetics*.
- Stentz, A. (1994/1995). Optimal and efficient path planning for partially-known environments / The Focussed D\* algorithm. ICRA / *J. Robotics and Automation*.
- Koenig, S., & Likhachev, M. (2002). D\* Lite. *AAAI*.
- Koenig, S., & Likhachev, M. (2004). Incremental A\*. *NIPS* (LPA\*).
- Koenig, S., & Likhachev, M. (2005). Fast replanning for navigation in unknown terrain. *IEEE Trans. Robotics* 21(3). (D\* Lite, TRO version with the `km` trick.)
- Likhachev, M., Gordon, G., & Thrun, S. (2003). ARA\*: Anytime A\* with provable bounds on sub-optimality. *NIPS*.
- Likhachev, M., Ferguson, D., Gordon, G., Stentz, A., & Thrun, S. (2005). Anytime Dynamic A\*. *ICAPS*. Also *Artificial Intelligence* 172 (2008).
- Harabor, D., & Grastien, A. (2011). Online graph pruning for pathfinding on grid maps. *AAAI*. (JPS)
- Nash, A., Daniel, K., Koenig, S., & Felner, A. (2007). Theta\*: Any-angle path planning on grids. *AAAI*.
- Ferguson, D., & Stentz, A. (2006). Using interpolation to improve path planning: The Field D\* algorithm. *J. Field Robotics*.
- Botea, A., Müller, M., & Schaeffer, J. (2004). Near optimal hierarchical path-finding. *J. Game Development*. (HPA\*)
- Korf, R. E. (1990). Real-time heuristic search. *Artificial Intelligence*. (LRTA\*)
- Harabor, D., Grastien, A., Öz, D., & Aksakalli, V. (2016). Optimal any-angle pathfinding in practice. *JAIR*. (Anya)

## Continuous / sampling

- Lozano-Pérez, T., & Wesley, M. A. (1979). An algorithm for planning collision-free paths among polyhedral obstacles. *CACM*. (visibility graphs)
- Kavraki, L. E., Svestka, P., Latombe, J.-C., & Overmars, M. (1996). Probabilistic roadmaps for path planning in high-dimensional configuration spaces. *IEEE TRA*. (PRM)
- LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning. TR 98-11, Iowa State. (RRT)
- Kuffner, J. J., & LaValle, S. M. (2000). RRT-Connect. *ICRA*.
- Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *IJRR*. (RRT\*, PRM\*)
- Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. Informed RRT\*; BIT\* (Batch Informed Trees).
- Otte, M., & Frazzoli, E. (2014/2016). RRTX. *WAFR* / *IJRR* 35(7).
- Phillips, M., & Likhachev, M. (2011). SIPP: Safe interval path planning for dynamic environments. *ICRA*.

## RTS swarm / cache / crowds / local avoidance

- Treuille, A., Cooper, S., & Popović, Z. (2006). Continuum crowds. *SIGGRAPH*.
- Emerson, E. (2013). Crowd pathfinding and steering using flow field tiles. In *Game AI Pro*, ch. 23.
- Riot Games. The tech behind Swarm. (Dijkstra maps, collision stamps, incremental goal updates.)
- van den Berg, J., Lin, M., & Manocha, D. (2008). Reciprocal velocity obstacles. *ICRA*. (RVO)
- van den Berg, J., Guy, S. J., Lin, M., & Manocha, D. (2011). Reciprocal n-body collision avoidance. *ISRR*. (ORCA)

## Fog of war / unknown terrain / stigmergy

- Lumelsky, V. J., & Stepanov, A. A. (1987). Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape. *Algorithmica*. (Bug)
- Kamon, I., Rimon, E., & Rivlin, E. TangentBug / DistBug (range-sensor Bug variants).
- Koenig & Likhachev (2005), above — freespace assumption *is* FOW planning.
- Dorigo, M., et al. Ant Colony Optimization (metaheuristic on a known graph — not used as a planner here).
- Stigmergic swarms: e.g. Kilobot pheromone coverage, *Royal Society Open Science* (2019); dual-pheromone exploration (SPACE, 2026). Useful for *search*, not for known-goal RTS pathing.

## MAPF (contrast only)

- Sharon, G., Stern, R., Felner, A., & Sturtevant, N. (2015). Conflict-based search for optimal multi-agent pathfinding. *Artificial Intelligence*.
- Wang, S., Xu, H., et al. (2025). Where paths collide: A comprehensive survey of classic and learning-based multi-agent pathfinding. arXiv:2505.19219.
- Stern, R., et al. (2019). Multi-agent pathfinding: Definitions, variants, and benchmarks.
