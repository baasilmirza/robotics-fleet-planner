package planner

import (
	"container/heap"
	"errors"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

// ErrNoPath is returned when no conflict-free path exists within the horizon.
var ErrNoPath = errors.New("planner: no path found")

type astate struct {
	c gridmap.Coord
	t int
}

type aitem struct {
	s    astate
	f, g int
}

type apq []*aitem

func (p apq) Len() int { return len(p) }
func (p apq) Less(i, j int) bool {
	a, b := p[i], p[j]
	if a.f != b.f {
		return a.f < b.f
	}
	// Prefer states closer to the goal (smaller g means larger remaining h).
	if a.g != b.g {
		return a.g > b.g
	}
	if a.s.t != b.s.t {
		return a.s.t < b.s.t
	}
	if a.s.c.Y != b.s.c.Y {
		return a.s.c.Y < b.s.c.Y
	}
	return a.s.c.X < b.s.c.X
}
func (p apq) Swap(i, j int) { p[i], p[j] = p[j], p[i] }
func (p *apq) Push(x interface{}) {
	*p = append(*p, x.(*aitem))
}
func (p *apq) Pop() interface{} {
	old := *p
	n := len(old)
	it := old[n-1]
	old[n-1] = nil
	*p = old[:n-1]
	return it
}

// AStar plans a shortest collision-free path for one robot from start to goal
// in the time-expanded graph induced by con. A robot may move to an adjacent
// cell or wait in place each tick. Earlier agents' reservations in con make the
// resulting path safe against them.
//
// The returned path is indexed by tick: path[0] == start, and path[k] is the
// robot's cell at startTick+k. horizon bounds the number of ticks considered;
// if the goal cannot be reached within it, ErrNoPath is returned.
func AStar(g *gridmap.Grid, start, goal gridmap.Coord, con *Constraints, horizon int) ([]gridmap.Coord, error) {
	if !g.Walkable(start) || !g.Walkable(goal) {
		return nil, ErrNoPath
	}
	if start == goal {
		return []gridmap.Coord{start}, nil
	}
	if con == nil {
		con = NewConstraints()
	}
	open := &apq{}
	heap.Init(open)
	gScore := make(map[astate]int)
	came := make(map[astate]astate)
	s0 := astate{start, 0}
	gScore[s0] = 0
	heap.Push(open, &aitem{s: s0, f: start.Manhattan(goal), g: 0})

	for open.Len() > 0 {
		cur := heap.Pop(open).(*aitem)
		if cur.g > gScore[cur.s] {
			continue
		}
		if cur.s.c == goal {
			return reconstruct(came, cur.s), nil
		}
		if cur.s.t >= horizon {
			continue
		}
		nt := cur.s.t + 1
		// Move actions.
		for _, d := range gridmap.Dirs {
			nb := cur.s.c.Add(d)
			if !g.Walkable(nb) {
				continue
			}
			if con.HasVertex(VertexKey{C: nb, T: nt}) {
				continue
			}
			if con.HasEdge(EdgeKey{From: cur.s.c, To: nb, T: cur.s.t}) {
				continue
			}
			relax(open, gScore, came, cur.s, astate{nb, nt}, cur.g+1, goal)
		}
		// Wait action.
		if !con.HasVertex(VertexKey{C: cur.s.c, T: nt}) {
			relax(open, gScore, came, cur.s, astate{cur.s.c, nt}, cur.g+1, goal)
		}
	}
	return nil, ErrNoPath
}

func relax(open *apq, gScore map[astate]int, came map[astate]astate, from, to astate, ng int, goal gridmap.Coord) {
	if old, ok := gScore[to]; ok && ng >= old {
		return
	}
	gScore[to] = ng
	came[to] = from
	heap.Push(open, &aitem{s: to, f: ng + to.c.Manhattan(goal), g: ng})
}

func reconstruct(came map[astate]astate, end astate) []gridmap.Coord {
	var rev []gridmap.Coord
	cur := end
	for {
		rev = append(rev, cur.c)
		prev, ok := came[cur]
		if !ok {
			break
		}
		cur = prev
	}
	path := make([]gridmap.Coord, len(rev))
	for i := range rev {
		path[len(rev)-1-i] = rev[i]
	}
	return path
}

// PathCost returns the number of moves (transitions) along a path.
func PathCost(path []gridmap.Coord) int {
	if len(path) == 0 {
		return 0
	}
	return len(path) - 1
}
