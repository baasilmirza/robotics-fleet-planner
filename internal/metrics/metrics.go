// Package metrics records fleet-run counters and derives the summary numbers
// reported per run: makespan, sum-of-costs, throughput, replan rate and
// deadlock count.
package metrics

import (
	"fmt"
	"sync"
	"time"
)

// Metrics is a concurrency-safe collector for a single fleet run.
type Metrics struct {
	mu sync.Mutex

	Ticks             int
	TasksTotal        int
	TasksCompleted    int
	TasksFailed       int
	SumOfCosts        int
	Replans           int
	ConflictsResolved int
	Deadlocks         int
	Collisions        int

	start    time.Time
	duration time.Duration
}

// Start marks the beginning of the run.
func (m *Metrics) Start() {
	m.mu.Lock()
	m.start = time.Now()
	m.mu.Unlock()
}

// Stop records the run duration.
func (m *Metrics) Stop() {
	m.mu.Lock()
	m.duration = time.Since(m.start)
	m.mu.Unlock()
}

// AddTicks accumulates elapsed ticks.
func (m *Metrics) AddTicks(n int) {
	m.mu.Lock()
	m.Ticks += n
	m.mu.Unlock()
}

// Inc increments a named counter. Unknown names are ignored.
func (m *Metrics) Inc(name string, delta int) {
	m.mu.Lock()
	defer m.mu.Unlock()
	switch name {
	case "tasks_total":
		m.TasksTotal += delta
	case "tasks_completed":
		m.TasksCompleted += delta
	case "tasks_failed":
		m.TasksFailed += delta
	case "replans":
		m.Replans += delta
	case "conflicts":
		m.ConflictsResolved += delta
	case "deadlocks":
		m.Deadlocks += delta
	case "collisions":
		m.Collisions += delta
	case "cost":
		m.SumOfCosts += delta
	}
}

// Snapshot is an immutable copy of the collected counters.
type Snapshot struct {
	Ticks             int
	TasksTotal        int
	TasksCompleted    int
	TasksFailed       int
	SumOfCosts        int
	Replans           int
	ConflictsResolved int
	Deadlocks         int
	Collisions        int
	Duration          time.Duration
}

// Snapshot returns a consistent copy of the counters.
func (m *Metrics) Snapshot() Snapshot {
	m.mu.Lock()
	defer m.mu.Unlock()
	return Snapshot{
		Ticks:             m.Ticks,
		TasksTotal:        m.TasksTotal,
		TasksCompleted:    m.TasksCompleted,
		TasksFailed:       m.TasksFailed,
		SumOfCosts:        m.SumOfCosts,
		Replans:           m.Replans,
		ConflictsResolved: m.ConflictsResolved,
		Deadlocks:         m.Deadlocks,
		Collisions:        m.Collisions,
		Duration:          m.duration,
	}
}

// SuccessRate returns completed tasks as a fraction of the total.
func (s Snapshot) SuccessRate() float64 {
	if s.TasksTotal == 0 {
		return 1
	}
	return float64(s.TasksCompleted) / float64(s.TasksTotal)
}

// Throughput returns completed tasks per minute given the wall-clock duration.
func (s Snapshot) Throughput() float64 {
	if s.Duration <= 0 {
		return 0
	}
	return float64(s.TasksCompleted) / s.Duration.Minutes()
}

// ReplansPerSec returns replans per wall-clock second.
func (s Snapshot) ReplansPerSec() float64 {
	if s.Duration <= 0 {
		return 0
	}
	return float64(s.Replans) / s.Duration.Seconds()
}

// String renders a human-readable multi-line summary.
func (s Snapshot) String() string {
	return fmt.Sprintf(
		"ticks=%d tasks=%d/%d failed=%d success=%.0f%% cost=%d replans=%d conflicts=%d deadlocks=%d collisions=%d throughput=%.1f tasks/min wall=%s",
		s.Ticks, s.TasksCompleted, s.TasksTotal, s.TasksFailed,
		s.SuccessRate()*100, s.SumOfCosts, s.Replans, s.ConflictsResolved,
		s.Deadlocks, s.Collisions, s.Throughput(), s.Duration.Round(time.Millisecond),
	)
}
