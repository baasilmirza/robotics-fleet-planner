package coordinator

import (
	"testing"
	"time"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

func TestFleetNoCollisions(t *testing.T) {
	cases := []struct {
		name          string
		size          int
		robots, tasks int
	}{
		{"small", 12, 3, 8},
		{"medium", 20, 6, 18},
		{"many-robots", 24, 10, 30},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			g := gridmap.Generate(tc.size, tc.size, 0.05, 42, tc.robots, 4)
			cfg := Config{
				Grid:              g,
				Robots:            tc.robots,
				Tasks:             tc.tasks,
				MaxTicks:          30000,
				Seed:              42,
				TickDuration:      time.Millisecond,
				DeadlockThreshold: 30,
			}
			snap := New(cfg).Run()
			if snap.Collisions != 0 {
				t.Fatalf("collisions = %d, want 0", snap.Collisions)
			}
			if snap.TasksCompleted != snap.TasksTotal {
				t.Fatalf("completed %d/%d tasks (failed %d, ticks %d)",
					snap.TasksCompleted, snap.TasksTotal, snap.TasksFailed, snap.Ticks)
			}
		})
	}
}

func TestFleetNoCollisionsOnFixtureMap(t *testing.T) {
	g, err := gridmap.Load("../../testdata/maps/warehouse-02.txt")
	if err != nil {
		t.Fatalf("load map: %v", err)
	}
	cfg := Config{
		Grid:              g,
		Robots:            4,
		Tasks:             12,
		MaxTicks:          30000,
		Seed:              7,
		TickDuration:      time.Millisecond,
		DeadlockThreshold: 30,
	}
	snap := New(cfg).Run()
	if snap.Collisions != 0 {
		t.Fatalf("collisions = %d, want 0", snap.Collisions)
	}
	if snap.TasksCompleted != snap.TasksTotal {
		t.Fatalf("completed %d/%d tasks (failed %d)", snap.TasksCompleted, snap.TasksTotal, snap.TasksFailed)
	}
}

func TestDynamicBlockageTriggersReplan(t *testing.T) {
	g := gridmap.Generate(24, 24, 0.05, 99, 4, 4)
	cfg := Config{
		Grid:              g,
		Robots:            4,
		Tasks:             8,
		MaxTicks:          30000,
		Seed:              99,
		TickDuration:      time.Millisecond,
		Dynamic:           true,
		BlockProb:         1.0,
		BlockInterval:     10,
		DeadlockThreshold: 30,
	}
	snap := New(cfg).Run()
	if snap.Collisions != 0 {
		t.Fatalf("collisions = %d, want 0", snap.Collisions)
	}
	if snap.Replans < 2 {
		t.Fatalf("replans = %d, want >= 2 to demonstrate dynamic replanning", snap.Replans)
	}
}

func TestScheduledBlockage(t *testing.T) {
	g := gridmap.Generate(15, 15, 0.0, 5, 2, 2)
	cfg := Config{
		Grid:              g,
		Robots:            2,
		Tasks:             4,
		MaxTicks:          30000,
		Seed:              5,
		TickDuration:      time.Millisecond,
		DeadlockThreshold: 30,
		BlockSchedule:     map[int][]gridmap.Coord{3: {{X: 7, Y: 7}}},
	}
	snap := New(cfg).Run()
	if snap.Collisions != 0 {
		t.Fatalf("collisions = %d, want 0", snap.Collisions)
	}
	if snap.Replans < 2 {
		t.Fatalf("replans = %d, want >= 2 after scheduled blockage", snap.Replans)
	}
}
