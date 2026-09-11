// Command fleet runs the warehouse fleet planner: a single animated demo or a
// benchmark sweep across grid sizes and robot counts.
package main

import (
	"flag"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/baasilmirza/robotics-fleet-planner/internal/coordinator"
	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
	"github.com/baasilmirza/robotics-fleet-planner/internal/render"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "fleet:", err)
		os.Exit(1)
	}
}

type options struct {
	mapPath      string
	robots       string
	tasks        int
	tick         time.Duration
	render       string
	maxTicks     int
	seed         int64
	deadlock     int
	dynamic      bool
	blockProb    float64
	blockEvery   int
	svgDir       string
	bench        bool
	grids        string
	size         int
	wallProb     float64
	verbose      bool
}

func run() error {
	var o options
	flag.StringVar(&o.mapPath, "map", "", "map file to load (omit to synthesize one)")
	flag.StringVar(&o.robots, "robots", "8", "robot count, or CSV list in --bench mode")
	flag.IntVar(&o.tasks, "tasks", 40, "number of tasks")
	flag.DurationVar(&o.tick, "tick", 50*time.Millisecond, "tick duration (affects throughput and animation speed)")
	flag.StringVar(&o.render, "render", "none", "renderer: none|ascii")
	flag.IntVar(&o.maxTicks, "max-ticks", 20000, "hard tick cap")
	flag.Int64Var(&o.seed, "seed", 1, "random seed")
	flag.IntVar(&o.deadlock, "deadlock", 40, "ticks without progress before deadlock resolution")
	flag.BoolVar(&o.dynamic, "dynamic", false, "randomly block cells during the run")
	flag.Float64Var(&o.blockProb, "block-prob", 0.5, "probability of a block event when one is due")
	flag.IntVar(&o.blockEvery, "block-every", 50, "ticks between dynamic block attempts")
	flag.StringVar(&o.svgDir, "svg", "", "directory for SVG frame export")
	flag.BoolVar(&o.bench, "bench", false, "run a benchmark sweep instead of a single demo")
	flag.StringVar(&o.grids, "grids", "20,40,80", "grid sizes (CSV) for --bench")
	flag.IntVar(&o.size, "size", 20, "synthetic grid size for a single run without --map")
	flag.Float64Var(&o.wallProb, "wall-prob", 0.12, "interior wall probability for synthetic grids")
	flag.BoolVar(&o.verbose, "verbose", false, "print per-tick status")
	flag.Parse()

	if o.bench {
		return runBench(o)
	}
	return runSingle(o)
}

func runSingle(o options) error {
	g, err := loadGrid(o)
	if err != nil {
		return err
	}
	var r coordinator.Renderer
	if o.render == "ascii" {
		r = render.NewASCII(os.Stdout, true)
	}
	fmt.Println("map:", g.Name, fmt.Sprintf("%dx%d", g.W, g.H), "starts:", len(g.Starts), "stations:", len(g.Stations))
	fmt.Println(render.Legend())
	cfg := coordinator.Config{
		Grid:              g,
		Robots:            atoiDefault(o.robots, 8),
		Tasks:             o.tasks,
		MaxTicks:          o.maxTicks,
		Seed:              o.seed,
		TickDuration:      o.tick,
		Dynamic:           o.dynamic,
		BlockProb:         o.blockProb,
		BlockInterval:     o.blockEvery,
		DeadlockThreshold: o.deadlock,
		Renderer:          r,
		SVGDir:            o.svgDir,
		Verbose:           o.verbose,
	}
	snap := coordinator.New(cfg).Run()
	fmt.Println(snap.String())
	return nil
}

func runBench(o options) error {
	robotsList := parseIntList(o.robots)
	gridList := parseIntList(o.grids)
	if len(robotsList) == 0 {
		robotsList = []int{8}
	}
	if len(gridList) == 0 {
		gridList = []int{20}
	}
	fmt.Println("| grid | robots | tasks | ticks | done | failed | cost | replans | deadlocks | collisions | success | throughput |")
	fmt.Println("|------|--------|-------|-------|------|--------|------|---------|-----------|------------|---------|------------|")
	for _, size := range gridList {
		for _, n := range robotsList {
			tasks := n * 5
			g := gridmap.Generate(size, size, o.wallProb, o.seed, n, max(2, n/2))
			cfg := coordinator.Config{
				Grid:              g,
				Robots:            n,
				Tasks:             tasks,
				MaxTicks:          o.maxTicks,
				Seed:              o.seed,
				TickDuration:      o.tick,
				DeadlockThreshold: o.deadlock,
			}
			snap := coordinator.New(cfg).Run()
			fmt.Printf("| %d | %d | %d | %d | %d | %d | %d | %d | %d | %d | %.0f%% | %.1f |\n",
				size, n, tasks, snap.Ticks, snap.TasksCompleted, snap.TasksFailed,
				snap.SumOfCosts, snap.Replans, snap.Deadlocks, snap.Collisions,
				snap.SuccessRate()*100, snap.Throughput())
		}
	}
	return nil
}

func loadGrid(o options) (*gridmap.Grid, error) {
	if o.mapPath != "" {
		return gridmap.Load(o.mapPath)
	}
	n := atoiDefault(o.robots, 8)
	g := gridmap.Generate(o.size, o.size, o.wallProb, o.seed, n, max(2, n/2))
	g.Name = fmt.Sprintf("synthetic-%dx%d", o.size, o.size)
	return g, nil
}

func parseIntList(s string) []int {
	var out []int
	for _, part := range strings.Split(s, ",") {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		v, err := strconv.Atoi(part)
		if err == nil && v > 0 {
			out = append(out, v)
		}
	}
	return out
}

func atoiDefault(s string, def int) int {
	v, err := strconv.Atoi(strings.TrimSpace(s))
	if err != nil || v < 1 {
		return def
	}
	return v
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}
