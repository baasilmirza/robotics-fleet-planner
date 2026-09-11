# Warehouse Fleet Planner — Go

## Problem

Fulfillment centers coordinate hundreds of drive units on a shared floor without
collisions or deadlocks. A fleet planner must assign tasks to idle robots, plan
each path through a dynamic environment, detect conflicts between agents, and
replan when a robot is blocked or a route becomes congested.

This project builds a grid-based multi-robot fleet planner in Go: task
allocation, single-agent path planning, prioritized multi-agent coordination,
conflict detection and resolution, and dynamic replanning — visualized and
benchmarked.

## Goals

- Parse a grid warehouse map (walls, pickup/dropoff stations, robot start poses).
- Single-agent planner: A* with a deterministic heuristic and tie-breaking.
- Multi-agent prioritized planning: plan robots sequentially; earlier robots'
  paths become time-windowed constraints for later robots.
- Conflict detection: vertex collisions (same cell, same tick) and edge
  collisions (swap across an edge in the same tick).
- Conflict resolution by replanning the lower-priority robot with constraints.
- Dynamic replanning: a blocked or congested robot triggers D* Lite or a
  constrained A* re-plan.
- Task allocation: nearest-idle-robot greedy baseline.
- Go concurrency: one goroutine per robot, a central coordinator, channels for
  commands/telemetry, `context` cancellation on shutdown.
- Rendering: terminal ASCII animation; optional SVG/PNG frame export.
- Metrics: makespan, sum-of-costs, success rate, throughput (tasks/min),
  replans/sec, deadlock count.
- Tests: table-driven planner tests, conflict-free invariant tests, `-race`,
  and benchmarks (N robots × grid size).
- CI: GitHub Actions — `go vet`, `go test -race`, `go test -bench`.

## Non-goals

- No real hardware and no ROS integration.
- No 3D perception or computer vision (separate concern).
- No machine learning.

## Architecture

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

- **Coordinator** owns the authoritative world state and advances discrete
  ticks. It allocates tasks, plans paths, and resolves conflicts.
- **Robots** are goroutines that receive a path/command over a channel, execute
  one step per tick, and report position/status back.
- **Renderer** subscribes to telemetry and draws each tick to the terminal
  (and optionally writes SVG frames).
- **Metrics** records timing and counts per run and prints a summary.

## Tech stack

- Go 1.22+
- Standard library: `container/heap`, `sync`, `context`, `flag`, `time`
- Optional: gRPC + Protocol Buffers for a state stream
- Optional: Prometheus client for `/metrics`
- Optional: Docker image for a self-contained demo

## Milestones

- **M1 — MVP:** map parse, A* planner, single robot, ASCII render.
- **M2 — Fleet:** task queue + greedy allocation, multiple robots, one goroutine
  per robot, tick loop.
- **M3 — Coordination:** prioritized planning, vertex/edge conflict detection
  and resolution.
- **M4 — Dynamic:** replanning (D* Lite or constrained A*), deadlock detection
  and resolution.
- **M5 — Proof:** metrics, benchmarks, GitHub Actions CI, README with GIF and
  a results table.

**Stretch:** CBS (Conflict-Based Search) optimal planner, gRPC state stream,
Prometheus metrics, Docker, SVG/PNG export.

## Acceptance criteria

- [ ] N robots complete M tasks with zero collisions (invariant test).
- [ ] Deadlock is detected and resolved, never hangs.
- [ ] Replan on dynamic blockage is demonstrated.
- [ ] `go test -race` is clean; benchmarks reported in the README.
- [ ] Metrics (makespan / sum-of-costs / throughput) printed per run.
- [ ] README with architecture, GIF, and results table.
- [ ] Public GitHub repo with green CI.

## Repository structure

```
cmd/fleet/main.go
internal/map/          # grid, loader, coordinates
internal/planner/      # astar.go, dstar.go, constraints.go
internal/coordinator/  # allocation, conflict, replan, tick loop
internal/robot/        # robot goroutine, state machine
internal/metrics/      # counters, timing, summary
internal/render/       # ascii.go, svg.go
testdata/maps/*.txt
docs/blog.md
.github/workflows/ci.yml
```

## Map format (example)

```
# grid: #=wall  .=free  S=station  @=robot start
###########
#S..@....S#
#.#######.#
#@.......@#
###########
```

## Example CLI

```
fleet --map testdata/maps/warehouse-01.txt --robots 8 --tasks 40 --tick 50ms
fleet --map ... --render ascii
fleet --map ... --bench --grids 20,40,80 --robots 4,8,16,32
```

## Resume bullets (post-build)

- "Built a multi-robot warehouse path planner in Go: A*/D* Lite planning,
  prioritized multi-agent conflict resolution, and dynamic replanning across
  concurrent robot goroutines."
- "Benchmarked fleet coordination across robot counts and grid sizes, tracking
  makespan, sum-of-costs, throughput, and deadlock rate; CI enforced race-clean
  tests."

## Time estimate

- MVP (M1) ~1 weekend.
- Full M1–M5 ~2–3 weeks part-time.
