package planner

import (
	"strconv"
	"testing"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

func benchGrid(w, h int) *gridmap.Grid {
	g := gridmap.NewGrid(w, h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			g.Set(gridmap.Coord{X: x, Y: y}, gridmap.Free)
		}
	}
	return g
}

func BenchmarkAStar(b *testing.B) {
	for _, size := range []int{20, 40, 80} {
		g := benchGrid(size, size)
		start := gridmap.Coord{X: 0, Y: 0}
		goal := gridmap.Coord{X: size - 1, Y: size - 1}
		b.Run(name(size), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				if _, err := AStar(g, start, goal, nil, size*4); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

func BenchmarkDStarLite(b *testing.B) {
	for _, size := range []int{20, 40, 80} {
		g := benchGrid(size, size)
		start := gridmap.Coord{X: 0, Y: 0}
		goal := gridmap.Coord{X: size - 1, Y: size - 1}
		b.Run(name(size), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				d := NewDStarLite(g, start, goal)
				d.ComputeShortestPath()
				if _, ok := d.ExtractPath(); !ok {
					b.Fatal("no path")
				}
			}
		})
	}
}

func name(n int) string { return strconv.Itoa(n) }
