package planner

import (
	"testing"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

func openGrid(w, h int) *gridmap.Grid {
	g := gridmap.NewGrid(w, h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			g.Set(gridmap.Coord{X: x, Y: y}, gridmap.Free)
		}
	}
	return g
}

func pathCost(p []gridmap.Coord) int { return PathCost(p) }

func TestAStarTable(t *testing.T) {
	tests := []struct {
		name     string
		w, h     int
		walls    []gridmap.Coord
		start    gridmap.Coord
		goal     gridmap.Coord
		wantCost int
		wantErr  bool
	}{
		{
			name: "straight line",
			w:    5, h: 1,
			start: gridmap.Coord{X: 0, Y: 0}, goal: gridmap.Coord{X: 4, Y: 0},
			wantCost: 4,
		},
		{
			name: "same cell",
			w:    3, h: 3,
			start: gridmap.Coord{X: 1, Y: 1}, goal: gridmap.Coord{X: 1, Y: 1},
			wantCost: 0,
		},
		{
			name: "detour around wall",
			w:    5, h: 3,
			walls: []gridmap.Coord{{X: 2, Y: 0}, {X: 2, Y: 1}},
			start: gridmap.Coord{X: 0, Y: 0}, goal: gridmap.Coord{X: 4, Y: 0},
			wantCost: 8,
		},
		{
			name: "unreachable",
			w:    5, h: 3,
			walls: []gridmap.Coord{{X: 2, Y: 0}, {X: 2, Y: 1}, {X: 2, Y: 2}},
			start: gridmap.Coord{X: 0, Y: 0}, goal: gridmap.Coord{X: 4, Y: 0},
			wantErr: true,
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			g := openGrid(tc.w, tc.h)
			for _, w := range tc.walls {
				g.Set(w, gridmap.Wall)
			}
			path, err := AStar(g, tc.start, tc.goal, nil, 1000)
			if tc.wantErr {
				if err == nil {
					t.Fatalf("expected error, got path %v", path)
				}
				return
			}
			if err != nil {
				t.Fatalf("AStar: %v", err)
			}
			if path[0] != tc.start || path[len(path)-1] != tc.goal {
				t.Fatalf("path endpoints %v..%v, want %v..%v", path[0], path[len(path)-1], tc.start, tc.goal)
			}
			if got := pathCost(path); got != tc.wantCost {
				t.Fatalf("cost = %d, want %d (path %v)", got, tc.wantCost, path)
			}
			assertContiguous(t, g, path)
		})
	}
}

func TestAStarVertexConstraintForcesWait(t *testing.T) {
	g := openGrid(2, 1)
	con := NewConstraints()
	con.AddVertex(VertexKey{C: gridmap.Coord{X: 1, Y: 0}, T: 1})
	path, err := AStar(g, gridmap.Coord{X: 0, Y: 0}, gridmap.Coord{X: 1, Y: 0}, con, 10)
	if err != nil {
		t.Fatalf("AStar: %v", err)
	}
	if pathCost(path) != 2 {
		t.Fatalf("cost = %d, want 2 (path %v)", pathCost(path), path)
	}
	if path[0] != path[1] {
		t.Fatalf("expected a wait at tick 1, path %v", path)
	}
}

func TestAStarEdgeConstraintForcesWait(t *testing.T) {
	g := openGrid(2, 1)
	con := NewConstraints()
	con.AddEdge(EdgeKey{From: gridmap.Coord{X: 0, Y: 0}, To: gridmap.Coord{X: 1, Y: 0}, T: 0})
	path, err := AStar(g, gridmap.Coord{X: 0, Y: 0}, gridmap.Coord{X: 1, Y: 0}, con, 10)
	if err != nil {
		t.Fatalf("AStar: %v", err)
	}
	if pathCost(path) != 2 {
		t.Fatalf("cost = %d, want 2 (path %v)", pathCost(path), path)
	}
}

func TestAStarOptimalOnOpenGrid(t *testing.T) {
	g := openGrid(20, 20)
	start := gridmap.Coord{X: 0, Y: 0}
	goal := gridmap.Coord{X: 19, Y: 19}
	path, err := AStar(g, start, goal, nil, 1000)
	if err != nil {
		t.Fatalf("AStar: %v", err)
	}
	if got, want := pathCost(path), start.Manhattan(goal); got != want {
		t.Fatalf("cost = %d, want optimal %d", got, want)
	}
}

func assertContiguous(t *testing.T, g *gridmap.Grid, path []gridmap.Coord) {
	t.Helper()
	for i, c := range path {
		if !g.Walkable(c) {
			t.Fatalf("path[%d]=%v not walkable", i, c)
		}
		if i == 0 {
			continue
		}
		if path[i-1].Manhattan(c) > 1 {
			t.Fatalf("path jumps from %v to %v", path[i-1], c)
		}
	}
}
