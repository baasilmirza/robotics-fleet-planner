# Warehouse Fleet Planner (Go)

A grid-based multi-robot fleet planner for a fulfillment-center floor: task
allocation, A* / D* Lite path planning, prioritized multi-agent coordination,
vertex and edge conflict resolution, dynamic replanning, and ASCII/SVG
visualization — built with one goroutine per robot and a central coordinator.

```
map loader ──> world state ──> task queue
                                   │
                          coordinator (goroutine)
                          ├─ allocate tasks (nearest idle robot)
                          ├─ plan per-robot paths (A* / D* Lite)
                          ├─ conflict check (vertex / edge)
                          ├─ replan on conflict or blockage
                          └─ tick loop ──> robot goroutines (step / act)
                                              │
                                     metrics + renderer (ASCII / SVG)
```

## Highlights

- **Single-agent planner** — time-expanded A* with a deterministic heuristic and
  tie-breaking, supporting wait actions and time-windowed constraints.
- **Prioritized multi-agent planning** — robots are planned in priority order;
  each accepted path becomes a reservation table for lower-priority robots.
- **Conflict handling** — vertex (same cell, same tick) and edge (swap across an
  edge in the same tick) conflicts are detected, and a hard interlock guarantees
  the fleet never actually collides.
- **Dynamic replanning** — D* Lite repairs a single robot's route around newly
  blocked cells; the coordinator replans the whole fleet when the world changes.
- **Deadlock detection** — a fleet-wide stall triggers priority re-ordering and
  replanning, and never hangs.
- **Concurrency** — one goroutine per robot, a central coordinator, per-robot
  command/telemetry channels, and `context` cancellation on shutdown. Clean
  under `go test -race`.
- **Metrics** — makespan, sum-of-costs, success rate, throughput, replan rate,
  deadlock and collision counts.

## Layout

```
cmd/fleet/main.go       CLI: single animated run or benchmark sweep
internal/map/           grid, coordinates, map loader, synthetic generator
internal/planner/       astar.go, dstar.go, constraints.go
internal/coordinator/   allocation, prioritized planning, conflict, replan, ticks
internal/robot/         per-robot goroutine and state machine
internal/metrics/       counters, timing, summary
internal/render/        ascii.go, svg.go
testdata/maps/*.txt     sample warehouse maps
docs/blog.md            design notes
.github/workflows/ci.yml
```

## Map format

```
# wall   . free   S station   @ robot start
###########
#S..@....S#
#.#######.#
#@.......@#
###########
```

## Usage

```sh
# animated single run
go run ./cmd/fleet --map testdata/maps/warehouse-02.txt --robots 4 --tasks 12 --render ascii

# synthetic map, export SVG frames
go run ./cmd/fleet --size 30 --robots 8 --tasks 40 --svg out/frames

# dynamic blockages during the run
go run ./cmd/fleet --size 30 --robots 8 --tasks 40 --dynamic --block-prob 0.5 --block-every 25

# benchmark sweep
go run ./cmd/fleet --bench --grids 20,40,80 --robots 4,8,16,32 --wall-prob 0.05
```

Flags: `--map`, `--robots`, `--tasks`, `--tick`, `--render`, `--max-ticks`,
`--seed`, `--deadlock`, `--dynamic`, `--block-prob`, `--block-every`, `--svg`,
`--bench`, `--grids`, `--size`, `--wall-prob`, `--verbose`.

## Results

Fleet sweep on synthetic maps (seed 1, 5% interior walls, one coordinator tick
= 50 ms; measured on an Intel i9-13900H):

| grid | robots | tasks | ticks (makespan) | cost | replans | deadlocks | collisions | success | throughput |
|------|--------|-------|------------------|------|---------|-----------|------------|---------|------------|
| 20×20 | 4  | 20 | 247 | 917  | 32  | 0 | 0 | 100% | 4027 tasks/min |
| 20×20 | 8  | 40 | 171 | 1178 | 68  | 0 | 0 | 100% | 7124 tasks/min |
| 20×20 | 16 | 80 | 141 | 1933 | 97  | 0 | 0 | 100% | 8526 tasks/min |
| 40×40 | 4  | 20 | 291 | 1055 | 37  | 0 | 0 | 100% | 751 tasks/min  |
| 40×40 | 8  | 40 | 352 | 2344 | 75  | 0 | 0 | 100% | 1427 tasks/min |
| 40×40 | 16 | 80 | 325 | 4696 | 125 | 0 | 0 | 100% | 1697 tasks/min |

Every configuration completed all tasks with zero collisions and zero
deadlocks.

### Planner microbenchmarks

`go test -run '^$' -bench . -benchmem ./internal/planner/`

| benchmark | 20×20 | 40×40 | 80×80 |
|-----------|-------|-------|-------|
| A* | 41 µs/op | 85 µs/op | 177 µs/op |
| D* Lite | 761 µs/op | 3.2 ms/op | 13.6 ms/op |

A* is the hot path used by prioritized planning. D* Lite is the incremental
repair planner used when a single robot's route is invalidated by a new
blockage; it is measured planning from scratch for comparability.

## Testing

```sh
go vet ./...
go test -race ./...
go test -run '^$' -bench . -benchmem ./...
```

- Table-driven A* tests (optimal cost, detours, vertex/edge constraints).
- D* Lite tests compared against A* and around dynamic blocks.
- Fleet invariant tests: N robots complete M tasks with **zero collisions**,
  including a fixture map and scheduled/dynamic blockages.

## Design notes

See [docs/blog.md](docs/blog.md) for the planning model, the reservation table,
and how conflict detection and deadlock resolution work.

## License

MIT
