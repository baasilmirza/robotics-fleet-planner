# Design notes: Warehouse Fleet Planner

## Problem

Hundreds of drive units share a fulfillment-center floor. A fleet planner must
assign tasks, plan each route through a dynamic environment, keep agents from
colliding, and replan when routes are blocked. This project builds that stack in
Go around a discrete-tick simulation.

## World model

The map is a rectangular grid of walls, free cells, and stations, plus robot
start poses. Robots move one cell per tick (4-connected) or wait. Time is
discrete, so a robot's trajectory is a sequence of cells indexed by tick.

## Single-agent planning

`AStar` searches a time-expanded graph whose states are `(cell, tick)`. Moves are
to a walkable neighbour or a wait in place. The heuristic is Manhattan distance;
ties are broken deterministically by `(f, g, t, y, x)` so runs are reproducible.
The result is a trajectory `path[k]` giving the robot's cell at `startTick+k`.

## Prioritized multi-agent planning

`Coordinator.planAll` plans robots in priority order (a per-robot bias, then ID).
Each accepted path is added to a `Constraints` reservation table:

- a **vertex** reservation `(cell, tick)` forbids any lower-priority robot from
  occupying that cell at that tick;
- an **edge** reservation forbids the *reverse* transition across the same edge
  at the same tick, which is what prevents two robots swapping places.

Because every robot is replanned together against one table, the resulting path
set is conflict-free by construction. Stationary robots (idle, finished, or
unplannable) reserve their cell for the whole horizon so movers route around
them. If a robot cannot be planned, it is forced to hold and the fleet is
replanned with that hold in place, repeating until a consistent set is found.

## Task allocation

Tasks are pickup/dropoff pairs drawn from stations. A greedy
nearest-idle-robot baseline assigns each unassigned task to the closest free
robot; each task is then a two-leg route (to pickup, then to dropoff).

## Conflicts and deadlock

Two failure modes matter:

- **Vertex conflict** — two robots in the same cell at the same tick.
- **Edge conflict** — two robots traversing the same edge in opposite directions
  in the same tick.

Edge reservations eliminate both during planning. As a last line of defence the
coordinator re-checks the intended moves each tick; if a conflict is ever
predicted it freezes the tick, records the conflict, and replans — so an actual
collision never occurs.

**Deadlock** is detected as a fleet-wide stall: no robot has moved for
`--deadlock` ticks while tasks remain. The coordinator resolves it by bumping
every working robot's priority and forcing a fresh, differently ordered plan. If
a stall survives several resolution rounds, the lowest-priority task is
abandoned so the run always terminates. The tick cap (`--max-ticks`) is the
ultimate backstop.

## Dynamic replanning

`DStarLite` is an incremental planner for a single robot: `Block(cell)` marks a
new obstacle, `ComputeShortestPath` repairs only the affected g-values, and
`ExtractPath` returns the repaired route. The coordinator uses it to demonstrate
replanning around a blockage, and otherwise replans the fleet with constrained
A* whenever the world changes (new task, waypoint reached, blockage, conflict,
or stall).

## Concurrency

Each robot is a goroutine with a command channel and a telemetry channel. The
coordinator sends exactly one command per robot per tick, then waits on a
barrier for one telemetry message from each. Robot pose is guarded by a mutex,
and `context` cancellation plus a `WaitGroup` provide clean shutdown. The whole
suite passes `go test -race`.

## Metrics

The coordinator records makespan (ticks), sum-of-costs (total robot moves),
tasks completed/failed, success rate, throughput, replan count, conflicts
resolved, deadlocks, and collisions. `Snapshot` derives the reported numbers.
