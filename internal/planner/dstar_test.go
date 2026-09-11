package planner

import (
	"testing"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

func TestDStarLiteMatchesAStarOnOpenGrid(t *testing.T) {
	g := openGrid(15, 15)
	start := gridmap.Coord{X: 0, Y: 0}
	goal := gridmap.Coord{X: 14, Y: 14}
	d := NewDStarLite(g, start, goal)
	d.ComputeShortestPath()
	path, ok := d.ExtractPath()
	if !ok {
		t.Fatal("ExtractPath failed")
	}
	if got, want := PathCost(path), start.Manhattan(goal); got != want {
		t.Fatalf("cost = %d, want %d", got, want)
	}
	assertContiguous(t, g, path)
}

func TestDStarLiteReplansAroundBlock(t *testing.T) {
	g := openGrid(7, 3)
	start := gridmap.Coord{X: 0, Y: 1}
	goal := gridmap.Coord{X: 6, Y: 1}
	d := NewDStarLite(g, start, goal)
	d.ComputeShortestPath()
	before, ok := d.ExtractPath()
	if !ok {
		t.Fatal("initial path missing")
	}
	if PathCost(before) != 6 {
		t.Fatalf("initial cost = %d, want 6", PathCost(before))
	}

	// Block the straight corridor one cell ahead.
	d.Block(gridmap.Coord{X: 1, Y: 1})
	d.ComputeShortestPath()
	after, ok := d.ExtractPath()
	if !ok {
		t.Fatal("no path after block")
	}
	for _, c := range after {
		if c == (gridmap.Coord{X: 1, Y: 1}) {
			t.Fatalf("repaired path still crosses blocked cell: %v", after)
		}
	}
	assertContiguous(t, g, after)
	if PathCost(after) <= PathCost(before) {
		t.Fatalf("expected longer detour, got %d vs %d", PathCost(after), PathCost(before))
	}
}

func TestDStarLiteUnreachable(t *testing.T) {
	g := openGrid(5, 3)
	for y := 0; y < 3; y++ {
		g.Set(gridmap.Coord{X: 2, Y: y}, gridmap.Wall)
	}
	d := NewDStarLite(g, gridmap.Coord{X: 0, Y: 1}, gridmap.Coord{X: 4, Y: 1})
	d.ComputeShortestPath()
	if _, ok := d.ExtractPath(); ok {
		t.Fatal("expected no path across a full wall")
	}
}

func TestDStarLiteMoveStart(t *testing.T) {
	g := openGrid(10, 10)
	goal := gridmap.Coord{X: 9, Y: 9}
	d := NewDStarLite(g, gridmap.Coord{X: 0, Y: 0}, goal)
	d.ComputeShortestPath()
	d.MoveStart(gridmap.Coord{X: 3, Y: 3})
	d.ComputeShortestPath()
	path, ok := d.ExtractPath()
	if !ok {
		t.Fatal("ExtractPath failed after MoveStart")
	}
	if path[0] != (gridmap.Coord{X: 3, Y: 3}) {
		t.Fatalf("path starts at %v, want (3,3)", path[0])
	}
	if got, want := PathCost(path), 12; got != want {
		t.Fatalf("cost = %d, want %d", got, want)
	}
}
