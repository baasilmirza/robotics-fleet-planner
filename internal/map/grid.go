// Package gridmap parses warehouse floor plans into a grid world used by the
// fleet planner. A map file uses four characters:
//
//	#  wall
//	.  free cell
//	S  pickup/dropoff station
//	@  robot start pose
package gridmap

import (
	"bufio"
	"fmt"
	"io"
	"math/rand"
	"os"
	"strings"
)

// Cell is the static terrain type of a grid cell.
type Cell uint8

const (
	// Wall is impassable terrain.
	Wall Cell = iota
	// Free is a walkable empty cell.
	Free
	// Station is a walkable pickup/dropoff station.
	Station
)

// Coord is an integer grid coordinate. X grows right, Y grows down.
type Coord struct {
	X, Y int
}

// Dirs is the deterministic neighbour order: north, east, south, west.
var Dirs = [...]Coord{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}

// Add returns c translated by o.
func (c Coord) Add(o Coord) Coord { return Coord{c.X + o.X, c.Y + o.Y} }

// Manhattan returns the Manhattan distance between c and o.
func (c Coord) Manhattan(o Coord) int { return abs(c.X-o.X) + abs(c.Y-o.Y) }

func (c Coord) String() string { return fmt.Sprintf("(%d,%d)", c.X, c.Y) }

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// Grid is an immutable terrain layout plus the start poses and stations parsed
// from a map. Dynamic obstacles added at runtime live in the coordinator, not
// here, so a Grid can be shared read-only.
type Grid struct {
	W, H     int
	Name     string
	Starts   []Coord
	Stations []Coord
	cells    []Cell
}

// NewGrid returns an all-free grid of the given size.
func NewGrid(w, h int) *Grid {
	return &Grid{W: w, H: h, cells: make([]Cell, w*h)}
}

// Set assigns a terrain cell. Out-of-bounds writes are ignored.
func (g *Grid) Set(c Coord, cell Cell) {
	if g.InBounds(c) {
		g.cells[c.Y*g.W+c.X] = cell
	}
}

// At returns the terrain at c, or Wall if c is out of bounds.
func (g *Grid) At(c Coord) Cell {
	if !g.InBounds(c) {
		return Wall
	}
	return g.cells[c.Y*g.W+c.X]
}

// InBounds reports whether c lies inside the grid.
func (g *Grid) InBounds(c Coord) bool {
	return c.X >= 0 && c.Y >= 0 && c.X < g.W && c.Y < g.H
}

// Walkable reports whether a robot may occupy c.
func (g *Grid) Walkable(c Coord) bool {
	return g.InBounds(c) && g.At(c) != Wall
}

// Neighbors returns the walkable neighbours of c in Dirs order.
func (g *Grid) Neighbors(c Coord) []Coord {
	out := make([]Coord, 0, len(Dirs))
	for _, d := range Dirs {
		n := c.Add(d)
		if g.Walkable(n) {
			out = append(out, n)
		}
	}
	return out
}

// Clone returns a deep copy of the grid.
func (g *Grid) Clone() *Grid {
	c := &Grid{W: g.W, H: g.H, Name: g.Name}
	c.cells = append([]Cell(nil), g.cells...)
	c.Starts = append([]Coord(nil), g.Starts...)
	c.Stations = append([]Coord(nil), g.Stations...)
	return c
}

// Parse reads a map from r. name is stored on the returned grid for reporting.
func Parse(r io.Reader, name string) (*Grid, error) {
	sc := bufio.NewScanner(r)
	sc.Buffer(make([]byte, 64*1024), 4*1024*1024)
	var lines []string
	for sc.Scan() {
		line := strings.TrimRight(sc.Text(), "\r")
		if strings.TrimSpace(line) == "" {
			continue
		}
		lines = append(lines, line)
	}
	if err := sc.Err(); err != nil {
		return nil, err
	}
	if len(lines) == 0 {
		return nil, fmt.Errorf("gridmap: empty map")
	}
	w := len(lines[0])
	h := len(lines)
	g := &Grid{W: w, H: h, Name: name, cells: make([]Cell, w*h)}
	for y, line := range lines {
		if len(line) != w {
			return nil, fmt.Errorf("gridmap: row %d has width %d, want %d", y, len(line), w)
		}
		for x := 0; x < w; x++ {
			c := Coord{X: x, Y: y}
			switch line[x] {
			case '#':
				g.Set(c, Wall)
			case '.':
				g.Set(c, Free)
			case 'S':
				g.Set(c, Station)
				g.Stations = append(g.Stations, c)
			case '@':
				g.Set(c, Free)
				g.Starts = append(g.Starts, c)
			default:
				return nil, fmt.Errorf("gridmap: unknown character %q at %d,%d", line[x], x, y)
			}
		}
	}
	return g, nil
}

// Load reads and parses a map file.
func Load(path string) (*Grid, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	return Parse(f, path)
}

// Generate builds a deterministic synthetic warehouse: a walled border, random
// interior walls, then stations and robot starts on distinct free cells.
func Generate(w, h int, wallProb float64, seed int64, numRobots, numStations int) *Grid {
	if w < 3 {
		w = 3
	}
	if h < 3 {
		h = 3
	}
	rng := rand.New(rand.NewSource(seed))
	g := NewGrid(w, h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := Coord{X: x, Y: y}
			if x == 0 || y == 0 || x == w-1 || y == h-1 {
				g.Set(c, Wall)
				continue
			}
			if rng.Float64() < wallProb {
				g.Set(c, Wall)
			} else {
				g.Set(c, Free)
			}
		}
	}
	var free []Coord
	for y := 1; y < h-1; y++ {
		for x := 1; x < w-1; x++ {
			c := Coord{X: x, Y: y}
			if g.At(c) == Free {
				free = append(free, c)
			}
		}
	}
	rng.Shuffle(len(free), func(i, j int) { free[i], free[j] = free[j], free[i] })
	i := 0
	for n := 0; n < numStations && i < len(free); n++ {
		g.Set(free[i], Station)
		g.Stations = append(g.Stations, free[i])
		i++
	}
	for n := 0; n < numRobots && i < len(free); n++ {
		g.Starts = append(g.Starts, free[i])
		i++
	}
	return g
}
