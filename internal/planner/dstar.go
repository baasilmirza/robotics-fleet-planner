package planner

import (
	"container/heap"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

// dInf is a large finite sentinel, kept well below MaxInt to avoid overflow
// when heuristics and path costs are added to it.
const dInf = 1 << 60

type dsItem struct {
	c      gridmap.Coord
	k1, k2 int
	idx    int
}

type dsHeap struct {
	items []*dsItem
	pos   map[gridmap.Coord]int
}

func (h *dsHeap) Len() int { return len(h.items) }
func (h *dsHeap) Less(i, j int) bool {
	a, b := h.items[i], h.items[j]
	if a.k1 != b.k1 {
		return a.k1 < b.k1
	}
	if a.k2 != b.k2 {
		return a.k2 < b.k2
	}
	if a.c.Y != b.c.Y {
		return a.c.Y < b.c.Y
	}
	return a.c.X < b.c.X
}
func (h *dsHeap) Swap(i, j int) {
	h.items[i], h.items[j] = h.items[j], h.items[i]
	h.items[i].idx = i
	h.items[j].idx = j
	h.pos[h.items[i].c] = i
	h.pos[h.items[j].c] = j
}
func (h *dsHeap) Push(x interface{}) {
	it := x.(*dsItem)
	it.idx = len(h.items)
	h.items = append(h.items, it)
	h.pos[it.c] = it.idx
}
func (h *dsHeap) Pop() interface{} {
	n := len(h.items)
	it := h.items[n-1]
	h.items = h.items[:n-1]
	delete(h.pos, it.c)
	it.idx = -1
	return it
}

// DStarLite is an incremental shortest-path planner for a single robot on a
// grid whose blocked cells can change at runtime. After a blockage is applied
// with Block, call ComputeShortestPath and ExtractPath to obtain a repaired
// route without planning from scratch. Distances are Manhattan, cost is one per
// step.
type DStarLite struct {
	grid    *gridmap.Grid
	start   gridmap.Coord
	goal    gridmap.Coord
	blocked map[gridmap.Coord]bool
	km      int
	g       map[gridmap.Coord]int
	rhs     map[gridmap.Coord]int
	open    *dsHeap
}

// NewDStarLite returns a planner from start to goal on g with no dynamic
// blockages.
func NewDStarLite(g *gridmap.Grid, start, goal gridmap.Coord) *DStarLite {
	d := &DStarLite{
		grid:    g,
		start:   start,
		goal:    goal,
		blocked: make(map[gridmap.Coord]bool),
		g:       make(map[gridmap.Coord]int),
		rhs:     make(map[gridmap.Coord]int),
		open:    &dsHeap{pos: make(map[gridmap.Coord]int)},
	}
	d.rhs[goal] = 0
	d.push(goal)
	return d
}

// Goal returns the current goal cell.
func (d *DStarLite) Goal() gridmap.Coord { return d.goal }

func (d *DStarLite) get(m map[gridmap.Coord]int, c gridmap.Coord) int {
	if v, ok := m[c]; ok {
		return v
	}
	return dInf
}

func (d *DStarLite) h(a, b gridmap.Coord) int { return a.Manhattan(b) }

func (d *DStarLite) calcKey(c gridmap.Coord) (int, int) {
	m := d.get(d.g, c)
	if r := d.get(d.rhs, c); r < m {
		m = r
	}
	return m + d.h(d.start, c) + d.km, m
}

func (d *DStarLite) passable(c gridmap.Coord) bool {
	return d.grid.Walkable(c) && !d.blocked[c]
}

func (d *DStarLite) succ(c gridmap.Coord) []gridmap.Coord {
	out := make([]gridmap.Coord, 0, len(gridmap.Dirs))
	for _, dir := range gridmap.Dirs {
		n := c.Add(dir)
		if d.passable(n) {
			out = append(out, n)
		}
	}
	return out
}

func (d *DStarLite) cost(u, v gridmap.Coord) int {
	if !d.passable(u) || !d.passable(v) {
		return dInf
	}
	return 1
}

func (d *DStarLite) push(c gridmap.Coord) {
	k1, k2 := d.calcKey(c)
	if i, ok := d.open.pos[c]; ok {
		d.open.items[i].k1 = k1
		d.open.items[i].k2 = k2
		heap.Fix(d.open, i)
		return
	}
	heap.Push(d.open, &dsItem{c: c, k1: k1, k2: k2})
}

func (d *DStarLite) remove(c gridmap.Coord) {
	if i, ok := d.open.pos[c]; ok {
		heap.Remove(d.open, i)
	}
}

func (d *DStarLite) updateVertex(u gridmap.Coord) {
	if u != d.goal {
		best := dInf
		for _, s := range d.succ(u) {
			c := d.cost(u, s)
			if c == dInf {
				continue
			}
			if v := c + d.get(d.g, s); v < best {
				best = v
			}
		}
		d.rhs[u] = best
	}
	d.remove(u)
	if d.get(d.g, u) != d.get(d.rhs, u) {
		d.push(u)
	}
}

// ComputeShortestPath repairs the g-values after the start or blockages change.
func (d *DStarLite) ComputeShortestPath() {
	for d.open.Len() > 0 {
		top := d.open.items[0]
		sk1, sk2 := d.calcKey(d.start)
		lessThanStart := top.k1 < sk1 || (top.k1 == sk1 && top.k2 < sk2)
		if !lessThanStart && d.get(d.rhs, d.start) == d.get(d.g, d.start) {
			break
		}
		it := heap.Pop(d.open).(*dsItem)
		u := it.c
		k1, k2 := it.k1, it.k2
		nk1, nk2 := d.calcKey(u)
		switch {
		case k1 < nk1 || (k1 == nk1 && k2 < nk2):
			d.push(u)
		case d.get(d.g, u) > d.get(d.rhs, u):
			d.g[u] = d.rhs[u]
			for _, s := range d.neighbors(u) {
				d.updateVertex(s)
			}
		default:
			d.g[u] = dInf
			for _, s := range d.neighbors(u) {
				d.updateVertex(s)
			}
		}
	}
}

func (d *DStarLite) neighbors(c gridmap.Coord) []gridmap.Coord {
	out := make([]gridmap.Coord, 0, len(gridmap.Dirs))
	for _, dir := range gridmap.Dirs {
		n := c.Add(dir)
		if d.grid.InBounds(n) {
			out = append(out, n)
		}
	}
	return out
}

// Block marks c as an obstacle and invalidates the affected edges.
func (d *DStarLite) Block(c gridmap.Coord) {
	d.blocked[c] = true
	d.updateVertex(c)
	for _, s := range d.neighbors(c) {
		d.updateVertex(s)
	}
}

// Unblock clears a previously applied dynamic obstacle.
func (d *DStarLite) Unblock(c gridmap.Coord) {
	delete(d.blocked, c)
	d.updateVertex(c)
	for _, s := range d.neighbors(c) {
		d.updateVertex(s)
	}
}

// MoveStart relocates the agent's current position without replanning.
func (d *DStarLite) MoveStart(newStart gridmap.Coord) {
	d.km += d.h(d.start, newStart)
	d.start = newStart
}

// ExtractPath greedily follows descending g-values from the start to the goal.
func (d *DStarLite) ExtractPath() ([]gridmap.Coord, bool) {
	if d.get(d.g, d.start) == dInf && d.get(d.rhs, d.start) == dInf {
		return nil, false
	}
	path := []gridmap.Coord{d.start}
	cur := d.start
	limit := d.grid.W*d.grid.H*4 + 16
	for cur != d.goal {
		bestVal := dInf
		var best gridmap.Coord
		found := false
		for _, s := range d.succ(cur) {
			v := d.cost(cur, s) + d.get(d.g, s)
			if v < bestVal {
				bestVal = v
				best = s
				found = true
			}
		}
		if !found || bestVal == dInf {
			return nil, false
		}
		path = append(path, best)
		cur = best
		if len(path) > limit {
			return nil, false
		}
	}
	return path, true
}
