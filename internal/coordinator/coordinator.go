// Package coordinator owns the authoritative world state for a fleet run. It
// advances discrete ticks, assigns tasks, plans conflict-free paths with
// prioritized planning, resolves conflicts by replanning, detects deadlock,
// and drives one goroutine per robot over channels.
package coordinator

import (
	"context"
	"fmt"
	"math/rand"
	"path/filepath"
	"sort"
	"sync"
	"time"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
	"github.com/baasilmirza/robotics-fleet-planner/internal/metrics"
	"github.com/baasilmirza/robotics-fleet-planner/internal/planner"
	"github.com/baasilmirza/robotics-fleet-planner/internal/render"
	"github.com/baasilmirza/robotics-fleet-planner/internal/robot"
)

// Renderer receives a frame per tick. render.ASCII satisfies this interface.
type Renderer interface {
	Frame(tick int, g *gridmap.Grid, pos map[int]gridmap.Coord, status string)
}

// Task is a single pickup/dropoff job.
type Task struct {
	ID      int
	Pickup  gridmap.Coord
	Dropoff gridmap.Coord
}

// Config configures a fleet run.
type Config struct {
	Grid              *gridmap.Grid
	Robots            int
	Tasks             int
	MaxTicks          int
	Seed              int64
	TickDuration      time.Duration
	Dynamic           bool
	BlockProb         float64
	BlockInterval     int
	BlockSchedule     map[int][]gridmap.Coord
	DeadlockThreshold int
	Renderer          Renderer
	SVGDir            string
	Verbose           bool
}

// Coordinator is a single fleet run.
type Coordinator struct {
	cfg     Config
	grid    *gridmap.Grid
	metrics *metrics.Metrics

	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup
	robots []*robot.Robot
	telChs []chan robot.Telemetry

	tasks      []Task
	taskDone   []bool
	robotTask  []int
	robotPhase []int
	robotGoal  []gridmap.Coord
	priority   []int
	attempts   []int

	pos       []gridmap.Coord
	paths     [][]gridmap.Coord
	pathStart []int

	steps      []int
	lastPos    []gridmap.Coord
	stuckTicks []int

	rng  *rand.Rand
	tick int
	dirty bool

	lastProgress int
	stallRounds  int
}

// New builds a coordinator, filling unset configuration with defaults.
func New(cfg Config) *Coordinator {
	if cfg.Robots < 1 {
		cfg.Robots = 1
	}
	if cfg.Tasks < 1 {
		cfg.Tasks = 1
	}
	if cfg.MaxTicks < 1 {
		cfg.MaxTicks = 10000
	}
	if cfg.TickDuration <= 0 {
		cfg.TickDuration = 50 * time.Millisecond
	}
	if cfg.DeadlockThreshold < 1 {
		cfg.DeadlockThreshold = 40
	}
	if cfg.BlockInterval < 1 {
		cfg.BlockInterval = 50
	}
	return &Coordinator{
		cfg:     cfg,
		grid:    cfg.Grid,
		metrics: &metrics.Metrics{},
		rng:     rand.New(rand.NewSource(cfg.Seed)),
	}
}

// Run executes the fleet until every task is finished, MaxTicks elapse, or the
// run is cancelled, then returns the collected metrics.
func (c *Coordinator) Run() metrics.Snapshot {
	c.ctx, c.cancel = context.WithCancel(context.Background())
	c.metrics.Start()
	c.spawnRobots()
	c.initWork()
	c.dirty = true

	tick := 0
	for ; tick < c.cfg.MaxTicks; tick++ {
		c.tick = tick
		c.assignTasks()
		c.replanIfDirty(tick)
		c.maybeDynamicBlock(tick)
		c.replanIfDirty(tick)
		c.step(tick)
		c.checkDeadlocks()
		c.renderFrame(tick)
		if c.allTasksDone() {
			tick++
			break
		}
	}
	c.shutdown()
	c.metrics.AddTicks(tick)
	c.metrics.Stop()
	return c.metrics.Snapshot()
}

// BlockCell adds a dynamic obstacle and forces a replan on the next tick. It is
// safe to call before Run to pre-seed blockages.
func (c *Coordinator) BlockCell(cell gridmap.Coord) {
	if !c.grid.Walkable(cell) {
		return
	}
	c.grid.Set(cell, gridmap.Wall)
	c.dirty = true
}

func (c *Coordinator) spawnRobots() {
	n := c.cfg.Robots
	starts := c.pickStarts(n)
	c.robots = make([]*robot.Robot, 0, n)
	c.telChs = make([]chan robot.Telemetry, 0, n)
	for i := 0; i < n; i++ {
		tel := make(chan robot.Telemetry, 1)
		r := robot.New(i, starts[i], tel, &c.wg)
		c.robots = append(c.robots, r)
		c.telChs = append(c.telChs, tel)
		c.wg.Add(1)
		go r.Run(c.ctx)
	}
	c.pos = make([]gridmap.Coord, n)
	c.lastPos = make([]gridmap.Coord, n)
	c.paths = make([][]gridmap.Coord, n)
	c.pathStart = make([]int, n)
	c.steps = make([]int, n)
	c.stuckTicks = make([]int, n)
	c.robotTask = make([]int, n)
	c.robotPhase = make([]int, n)
	c.robotGoal = make([]gridmap.Coord, n)
	c.priority = make([]int, n)
	c.attempts = make([]int, n)
	for i := range c.robots {
		c.pos[i] = starts[i]
		c.lastPos[i] = starts[i]
		c.robotTask[i] = -1
	}
}

func (c *Coordinator) initWork() {
	c.tasks = c.makeTasks(c.cfg.Tasks)
	c.taskDone = make([]bool, len(c.tasks))
	c.metrics.Inc("tasks_total", len(c.tasks))
}

func (c *Coordinator) pickStarts(n int) []gridmap.Coord {
	out := make([]gridmap.Coord, 0, n)
	used := map[gridmap.Coord]bool{}
	for _, s := range c.grid.Starts {
		if len(out) >= n {
			break
		}
		if c.grid.Walkable(s) && !used[s] {
			out = append(out, s)
			used[s] = true
		}
	}
	for y := 0; y < c.grid.H && len(out) < n; y++ {
		for x := 0; x < c.grid.W && len(out) < n; x++ {
			cell := gridmap.Coord{X: x, Y: y}
			if c.grid.Walkable(cell) && !used[cell] {
				out = append(out, cell)
				used[cell] = true
			}
		}
	}
	for len(out) < n {
		out = append(out, gridmap.Coord{})
	}
	return out
}

func (c *Coordinator) makeTasks(n int) []Task {
	var free []gridmap.Coord
	for y := 0; y < c.grid.H; y++ {
		for x := 0; x < c.grid.W; x++ {
			cell := gridmap.Coord{X: x, Y: y}
			if c.grid.Walkable(cell) {
				free = append(free, cell)
			}
		}
	}
	pick := func() gridmap.Coord {
		if len(c.grid.Stations) >= 2 {
			return c.grid.Stations[c.rng.Intn(len(c.grid.Stations))]
		}
		if len(free) > 0 {
			return free[c.rng.Intn(len(free))]
		}
		return gridmap.Coord{}
	}
	tasks := make([]Task, 0, n)
	for i := 0; i < n; i++ {
		p := pick()
		d := pick()
		for d == p && len(c.grid.Stations) > 1 {
			d = pick()
		}
		tasks = append(tasks, Task{ID: i, Pickup: p, Dropoff: d})
	}
	return tasks
}

func (c *Coordinator) assignTasks() {
	for ti := range c.tasks {
		if c.taskDone[ti] {
			continue
		}
		already := false
		for id := range c.robots {
			if c.robotTask[id] == ti {
				already = true
				break
			}
		}
		if already {
			continue
		}
		best := -1
		bestD := 1 << 30
		for id := range c.robots {
			if c.robotTask[id] >= 0 {
				continue
			}
			d := c.pos[id].Manhattan(c.tasks[ti].Pickup)
			if d < bestD || (d == bestD && id < best) {
				bestD = d
				best = id
			}
		}
		if best < 0 {
			return
		}
		c.robotTask[best] = ti
		c.robotPhase[best] = 0
		c.robotGoal[best] = c.tasks[ti].Pickup
		c.attempts[best] = 0
		c.dirty = true
	}
}

func (c *Coordinator) replanIfDirty(tick int) {
	if !c.dirty {
		return
	}
	c.planAll(tick)
	c.metrics.Inc("replans", 1)
	c.dirty = false
}

func (c *Coordinator) planOrder() []int {
	order := make([]int, 0, len(c.robots))
	for id := range c.robots {
		order = append(order, id)
	}
	sort.SliceStable(order, func(i, j int) bool {
		a, b := order[i], order[j]
		if c.priority[a] != c.priority[b] {
			return c.priority[a] > c.priority[b]
		}
		return a < b
	})
	return order
}

// planAll performs prioritized planning: robots are planned in priority order
// and each accepted path becomes a time-windowed constraint for the robots
// planned after it. Stationary robots (idle, finished, or unplannable) reserve
// their cells for the whole horizon so movers route around them. Because every
// robot is replanned together against one reservation table, the resulting path
// set is conflict-free by construction.
func (c *Coordinator) planAll(tick int) {
	hold := c.holdHorizon()
	forceHold := make(map[int]bool)
	for {
		con := planner.NewConstraints()
		targets := make([]gridmap.Coord, len(c.robots))
		parked := make(map[gridmap.Coord]bool)
		for id := range c.robots {
			cur := c.pos[id]
			switch {
			case forceHold[id]:
				targets[id] = cur
			case c.robotTask[id] >= 0:
				targets[id] = c.robotGoal[id]
			default:
				targets[id] = c.parkTarget(cur, parked)
				if targets[id] != cur {
					parked[targets[id]] = true
				}
			}
			if targets[id] == cur {
				con.AddHold(cur, 0, hold)
			}
		}
		newFail := false
		for _, id := range c.planOrder() {
			cur := c.pos[id]
			target := targets[id]
			if target == cur {
				c.paths[id] = []gridmap.Coord{cur}
				c.pathStart[id] = tick
				continue
			}
			h := c.horizon(cur, target)
			path, err := planner.AStar(c.grid, cur, target, con, h)
			if err != nil {
				if c.cfg.Verbose {
					fmt.Printf("plan fail: robot=%d from=%v to=%v task=%d err=%v\n",
						id, cur, target, c.robotTask[id], err)
				}
				forceHold[id] = true
				newFail = true
				c.paths[id] = []gridmap.Coord{cur}
				c.pathStart[id] = tick
				continue
			}
			c.paths[id] = path
			c.pathStart[id] = tick
			con.AddPath(path, 0)
			if c.robotTask[id] < 0 {
				// An idle robot does not trigger a replan when it arrives, so
				// reserve its parking spot for the rest of the horizon.
				con.AddHold(path[len(path)-1], len(path), hold)
			}
		}
		if !newFail {
			return
		}
	}
}

// parkTarget returns from unchanged unless it sits on a station, in which case
// it returns the nearest walkable non-station cell that is neither occupied nor
// claimed by another idle robot. Among reachable candidates it prefers the one
// with the fewest walkable neighbours (dead-ends first), so idle robots tuck
// out of corridors instead of blocking them.
func (c *Coordinator) parkTarget(from gridmap.Coord, claimed map[gridmap.Coord]bool) gridmap.Coord {
	if c.grid.At(from) != gridmap.Station {
		return from
	}
	occupied := make(map[gridmap.Coord]bool, len(c.pos))
	for _, p := range c.pos {
		occupied[p] = true
	}
	best := from
	bestDeg := 1 << 30
	bestDist := 1 << 30
	seen := map[gridmap.Coord]bool{from: true}
	dist := map[gridmap.Coord]int{from: 0}
	queue := []gridmap.Coord{from}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for _, d := range gridmap.Dirs {
			n := cur.Add(d)
			if !c.grid.Walkable(n) || seen[n] {
				continue
			}
			seen[n] = true
			dist[n] = dist[cur] + 1
			queue = append(queue, n)
			if c.grid.At(n) == gridmap.Station || occupied[n] || claimed[n] {
				continue
			}
			deg := len(c.grid.Neighbors(n))
			if deg < bestDeg || (deg == bestDeg && dist[n] < bestDist) {
				best, bestDeg, bestDist = n, deg, dist[n]
			}
		}
	}
	return best
}

func (c *Coordinator) horizon(a, b gridmap.Coord) int {
	return a.Manhattan(b)*4 + 2*len(c.robots) + 40
}

// holdHorizon is the longest time a stationary robot must reserve its cell: at
// least as large as the largest moving-robot planning horizon.
func (c *Coordinator) holdHorizon() int {
	return (c.grid.W+c.grid.H)*4 + 2*len(c.robots) + 40
}

func (c *Coordinator) step(tick int) {
	next := make([]gridmap.Coord, len(c.robots))
	moving := make([]bool, len(c.robots))
	final := make([]gridmap.Coord, len(c.robots))
	for id := range c.robots {
		p := c.paths[id]
		idx := tick - c.pathStart[id]
		n := c.pos[id]
		has := false
		if p != nil && idx >= 0 && idx < len(p) {
			if idx+1 < len(p) {
				n = p[idx+1]
				has = n != p[idx]
			} else {
				n = p[idx]
			}
			if p[idx] != c.pos[id] {
				c.dirty = true
			}
		}
		next[id] = n
		moving[id] = has
		if has {
			final[id] = n
		} else {
			final[id] = c.pos[id]
		}
	}
	if c.detectConflict(final, next, moving) {
		// Predicted conflict: hold every robot for this tick and replan before
		// the next one. This is the last line of defence that guarantees no two
		// robots ever occupy the same cell or swap edges at the same tick.
		c.metrics.Inc("conflicts", 1)
		for id := range c.robots {
			next[id] = c.pos[id]
			moving[id] = false
			final[id] = c.pos[id]
		}
		c.dirty = true
	}
	for id := range c.robots {
		c.robots[id].Commands() <- robot.Command{Next: next[id], HasNext: moving[id]}
	}
	for id := range c.robots {
		tel := <-c.telChs[id]
		if tel.Pos != c.pos[id] {
			c.steps[id]++
			c.metrics.Inc("cost", 1)
			c.lastProgress = tick
			c.stallRounds = 0
		}
		c.pos[id] = tel.Pos
	}
	for id := range c.robots {
		if c.robotTask[id] < 0 {
			c.stuckTicks[id] = 0
			continue
		}
		if c.pos[id] == c.lastPos[id] {
			c.stuckTicks[id]++
		} else {
			c.stuckTicks[id] = 0
		}
		c.lastPos[id] = c.pos[id]
		if c.pos[id] == c.robotGoal[id] {
			c.advancePhase(id)
		}
	}
}

func (c *Coordinator) detectConflict(final, next []gridmap.Coord, moving []bool) bool {
	seen := map[gridmap.Coord]int{}
	for id, cell := range final {
		if _, ok := seen[cell]; ok {
			return true
		}
		seen[cell] = id
	}
	for i := range c.robots {
		if !moving[i] {
			continue
		}
		for j := i + 1; j < len(c.robots); j++ {
			if !moving[j] {
				continue
			}
			if c.pos[i] == next[j] && c.pos[j] == next[i] {
				return true
			}
		}
	}
	return false
}

func (c *Coordinator) advancePhase(id int) {
	ti := c.robotTask[id]
	switch c.robotPhase[id] {
	case 0:
		c.robotPhase[id] = 1
		c.robotGoal[id] = c.tasks[ti].Dropoff
		c.dirty = true
	case 1:
		c.taskDone[ti] = true
		c.robotTask[id] = -1
		c.robotPhase[id] = 0
		c.priority[id] = 0
		c.metrics.Inc("tasks_completed", 1)
		c.dirty = true
	}
}

// checkDeadlocks detects a fleet-wide stall: no robot has moved for
// DeadlockThreshold ticks while tasks remain. It resolves the stall by bumping
// every working robot's planning priority, forcing a fresh, differently ordered
// plan. If the stall persists across several rounds the lowest-priority working
// task is abandoned so the run always terminates.
func (c *Coordinator) checkDeadlocks() {
	if c.allTasksDone() {
		return
	}
	if c.tick-c.lastProgress <= c.cfg.DeadlockThreshold {
		return
	}
	c.metrics.Inc("deadlocks", 1)
	c.stallRounds++
	for id := range c.robots {
		if c.robotTask[id] >= 0 {
			c.priority[id]++
		}
	}
	if c.stallRounds > 8 {
		victim := -1
		for id := range c.robots {
			if c.robotTask[id] >= 0 && (victim == -1 || c.priority[id] < c.priority[victim]) {
				victim = id
			}
		}
		if victim >= 0 {
			ti := c.robotTask[victim]
			if c.cfg.Verbose {
				fmt.Printf("deadlock: abandoning task %d (pickup %v dropoff %v) held by robot %d at %v\n",
					ti, c.tasks[ti].Pickup, c.tasks[ti].Dropoff, victim, c.pos[victim])
			}
			c.taskDone[ti] = true
			c.robotTask[victim] = -1
			c.robotPhase[victim] = 0
			c.metrics.Inc("tasks_failed", 1)
		}
		c.stallRounds = 0
	}
	c.lastProgress = c.tick
	c.dirty = true
}

func (c *Coordinator) maybeDynamicBlock(tick int) {
	if cells, ok := c.cfg.BlockSchedule[tick]; ok {
		for _, cell := range cells {
			c.BlockCell(cell)
		}
	}
	if !c.cfg.Dynamic || tick == 0 || tick%c.cfg.BlockInterval != 0 {
		return
	}
	if c.rng.Float64() > c.cfg.BlockProb {
		return
	}
	for tries := 0; tries < 50; tries++ {
		cell := gridmap.Coord{X: c.rng.Intn(c.grid.W), Y: c.rng.Intn(c.grid.H)}
		if !c.grid.Walkable(cell) || c.grid.At(cell) == gridmap.Station {
			continue
		}
		occupied := false
		for _, p := range c.pos {
			if p == cell {
				occupied = true
				break
			}
		}
		if occupied {
			continue
		}
		c.BlockCell(cell)
		return
	}
}

func (c *Coordinator) allTasksDone() bool {
	for _, done := range c.taskDone {
		if !done {
			return false
		}
	}
	return true
}

func (c *Coordinator) positionMap() map[int]gridmap.Coord {
	m := make(map[int]gridmap.Coord, len(c.robots))
	for id := range c.robots {
		m[id] = c.pos[id]
	}
	return m
}

func (c *Coordinator) statusLine() string {
	done := 0
	for _, d := range c.taskDone {
		if d {
			done++
		}
	}
	return fmt.Sprintf("tasks %d/%d", done, len(c.tasks))
}

func (c *Coordinator) renderFrame(tick int) {
	if c.cfg.Renderer != nil {
		c.cfg.Renderer.Frame(tick, c.grid, c.positionMap(), c.statusLine())
	}
	if c.cfg.SVGDir != "" {
		name := filepath.Join(c.cfg.SVGDir, fmt.Sprintf("frame-%04d.svg", tick))
		_ = render.WriteSVG(name, tick, c.grid, c.positionMap(), 20)
	}
}

func (c *Coordinator) shutdown() {
	c.cancel()
	for _, r := range c.robots {
		close(r.Commands())
	}
	c.wg.Wait()
}
