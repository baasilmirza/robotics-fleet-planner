package planner

import gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"

// VertexKey forbids a robot from occupying cell C at time T.
type VertexKey struct {
	C gridmap.Coord
	T int
}

// EdgeKey forbids the transition From->To between time T and T+1. It captures
// edge (swap) collisions, where two robots traverse the same edge in opposite
// directions during the same tick.
type EdgeKey struct {
	From, To gridmap.Coord
	T        int
}

// Constraints is a time-windowed reservation table accumulated from the paths
// of higher-priority robots. It is the interface between prioritized planning
// and the single-agent A* planner.
type Constraints struct {
	vertex map[VertexKey]struct{}
	edge   map[EdgeKey]struct{}
}

// NewConstraints returns an empty reservation table.
func NewConstraints() *Constraints {
	return &Constraints{
		vertex: make(map[VertexKey]struct{}),
		edge:   make(map[EdgeKey]struct{}),
	}
}

// AddVertex reserves cell C at time T.
func (c *Constraints) AddVertex(k VertexKey) { c.vertex[k] = struct{}{} }

// AddEdge reserves the transition From->To at time T.
func (c *Constraints) AddEdge(k EdgeKey) { c.edge[k] = struct{}{} }

// HasVertex reports whether cell C is reserved at time T.
func (c *Constraints) HasVertex(k VertexKey) bool {
	_, ok := c.vertex[k]
	return ok
}

// HasEdge reports whether the transition From->To is reserved at time T.
func (c *Constraints) HasEdge(k EdgeKey) bool {
	_, ok := c.edge[k]
	return ok
}

// VertexCount returns the number of vertex reservations.
func (c *Constraints) VertexCount() int { return len(c.vertex) }

// EdgeCount returns the number of edge reservations.
func (c *Constraints) EdgeCount() int { return len(c.edge) }

// AddPath reserves every vertex and edge of an absolute-time path whose first
// element occurs at startTick. For each move u->v at time t it adds a vertex
// reservation on v at t+1 and forbids the reverse transition v->u at time t,
// which is what prevents a lower-priority robot from swapping through the same
// edge during the same tick.
func (c *Constraints) AddPath(path []gridmap.Coord, startTick int) {
	for i, p := range path {
		t := startTick + i
		c.AddVertex(VertexKey{C: p, T: t})
		if i > 0 && path[i-1] != p {
			c.AddEdge(EdgeKey{From: p, To: path[i-1], T: t - 1})
		}
	}
}

// AddHold reserves a stationary robot's cell for every tick in [start, end].
func (c *Constraints) AddHold(at gridmap.Coord, start, end int) {
	for t := start; t <= end; t++ {
		c.AddVertex(VertexKey{C: at, T: t})
	}
}
