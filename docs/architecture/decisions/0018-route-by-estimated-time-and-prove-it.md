# 0018: Choose the route by estimated time, and prove it

- **Status:** Accepted
- **Date:** 2026-11-19

## Context

The fastest route is not the one with the fewest tiles: a pivot turn costs time, a long straight is
fast. In a host simulation with assumed constants (100 random 9x9 mazes), the fewest-tiles route was
slower than the fastest in 38 of them, by 2.1 s on average and 4.1 s at most. The robot also does not know the whole maze after trial 1: tiles it did not see may
hide a faster way ([0017](0017-explore-then-prove-then-run-the-proven-path.md)).
The map keeps no difference between an open edge and one never looked at yet (0014 listed that as
missing); this needs it.

## Decision

**Cost is time, not tiles.** A search state is a tile and a heading. An edge is either a pivot turn
(90 degrees `t90`; 180 degrees `t180 = 1.8 * t90`) or a straight of k tiles. A straight of distance d
at top speed v and acceleration a takes `d/v + v/a` when `d >= v*v/a`, and `2*sqrt(d/a)` otherwise;
every straight starts and ends at rest, which errs on the slow side.

- The constants (`v`, `a`, `t90`, the stop and start costs) come from the **calibrated robot tests**
  (#54), never from the times logged in an exploration, because exploring is slower and unlike the run.
- The plan is made with the fast settings, and trials 2 and 3 run the same path (0017).
- Integer arithmetic in 10 ms ticks and small tables, so the search is cheap on the AVR.
- A tie is broken by fewer turns, then by fewer edges the robot has not yet verified.
- The fewest-tiles and fewest-turns routes are only a comparison in the visualisation and the logs.

**The proof.** After the finish is first reached, take two maps of the same maze:

- `K`, the best known time: every edge never looked at is a wall.
- `L`, the best possible time: every edge never looked at is open.

`K >= L`. If `K == L` the route is proved the fastest. If not, only an unseen tile that a route faster
than `K` could still pass through can matter: a tile `t` is *relevant* when the best route through `t`
over the optimistic map is less than `K` (a bound over tile-and-heading segments, so a tile is
only relevant at the headings that can beat `K`). The robot goes to the nearest relevant unseen tile,
looks, updates the map, and asks again; each look moves `K` down, `L` up, or removes tiles from the
relevant set, so it ends. It also ends at the run limit, with the best known route.

On the robot the relevance test is a forward search from the finish and a backward search from the
start over the optimistic map (about three searches per decision), not a list of whole candidate
routes. The candidate routes (A, B, ...) in the explainer are a way for a person to see it.

**Where it runs.** The planner works on the real 9x9 maze (4 headings: 324 states, about 1.3 KB),
not on the start-relative explorer map ([0019](0019-section-b-map-17x9-folded-to-9x9.md)). Section A
is the 7x7 start-relative map the explorer already has; its planner needs the same known/unknown
difference.

## Consequences

- The map needs known versus unknown edges (#55, tile-count version first), then the time costing
  (#76) on top of it. #55's proof is the base; this ADR changes the cost, not the idea.
- The robot tests must measure `v`, `a`, `t90` and the stop cost at each speed level (#54), and trial 2
  checks the predicted time of the run against the actual one.
- The planner must run in a time the control loop can spare. Timing it on the AVR with `micros()` is a
  robot task (#54); until then the cost is estimated, not measured.
- Verified on the host only: 0 mismatches in 100 random mazes between the proved time and the true
  optimum, and nothing the robot did not see before the finish was used. The real timing and the
  calibrated constants are not measured.

## Alternatives considered

- **Fewest tiles (flood fill).** Simple, but not the fastest; kept only for comparison and as the
  base of #55.
- **Fewest turns.** Also not the fastest (long detours); comparison only.
- **Cost from the logged exploration times.** They are slower and noisy; the run does not behave
  like exploring.
- **List and compare whole candidate routes on the robot.** Too much memory and time for the same
  answer as the three searches.
