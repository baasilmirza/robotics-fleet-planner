// Package robot implements the per-robot goroutine. Each robot owns its pose,
// executes one coordinator command per tick, and reports telemetry back over a
// channel. All shared state is passed by channel or guarded by a mutex so the
// fleet runs clean under the race detector.
package robot

import (
	"context"
	"sync"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

// Status is a robot's reported activity for the current tick.
type Status int

const (
	// Idle means the robot has no assigned task.
	Idle Status = iota
	// Moving means the robot advanced to a new cell this tick.
	Moving
	// Waiting means the robot held position this tick.
	Waiting
	// Done means the robot has shut down.
	Done
)

func (s Status) String() string {
	switch s {
	case Idle:
		return "idle"
	case Moving:
		return "moving"
	case Waiting:
		return "waiting"
	case Done:
		return "done"
	}
	return "unknown"
}

// Command is a coordinator instruction for one tick. When HasNext is true the
// robot moves to Next; otherwise it holds position.
type Command struct {
	Next    gridmap.Coord
	HasNext bool
	Stop    bool
}

// Telemetry is the robot's post-tick report.
type Telemetry struct {
	ID     int
	Pos    gridmap.Coord
	Status Status
}

// Robot is a single drive unit running in its own goroutine.
type Robot struct {
	ID int

	cmd chan Command
	tel chan<- Telemetry
	wg  *sync.WaitGroup

	mu     sync.Mutex
	pos    gridmap.Coord
	status Status
}

// New creates a robot at start. tel is the coordinator's receive channel for
// this robot and wg tracks the goroutine's lifetime.
func New(id int, start gridmap.Coord, tel chan<- Telemetry, wg *sync.WaitGroup) *Robot {
	return &Robot{ID: id, pos: start, status: Idle, cmd: make(chan Command), tel: tel, wg: wg}
}

// Commands returns the channel the coordinator uses to send instructions.
func (r *Robot) Commands() chan<- Command { return r.cmd }

// Pos returns the robot's last reported pose.
func (r *Robot) Pos() gridmap.Coord {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.pos
}

// Status returns the robot's last reported status.
func (r *Robot) Status() Status {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.status
}

// Run is the robot's event loop. It exits on context cancellation, channel
// close, or a Stop command.
func (r *Robot) Run(ctx context.Context) {
	defer r.wg.Done()
	for {
		select {
		case <-ctx.Done():
			return
		case cmd, ok := <-r.cmd:
			if !ok {
				return
			}
			if cmd.Stop {
				r.set(Done, r.Pos())
				return
			}
			if cmd.HasNext {
				r.set(Moving, cmd.Next)
			} else {
				r.set(Waiting, r.Pos())
			}
		}
	}
}

func (r *Robot) set(s Status, p gridmap.Coord) {
	r.mu.Lock()
	r.pos = p
	r.status = s
	tel := Telemetry{ID: r.ID, Pos: p, Status: s}
	r.mu.Unlock()
	r.tel <- tel
}
