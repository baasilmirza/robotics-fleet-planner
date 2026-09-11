// Package render draws fleet state to a terminal (ASCII animation) or to
// standalone SVG frames.
package render

import (
	"fmt"
	"io"
	"sort"
	"strings"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

// ASCII renders frames to a writer, optionally clearing the screen first so the
// output animates in place.
type ASCII struct {
	out   io.Writer
	clear bool
}

// NewASCII returns an ASCII renderer writing to out.
func NewASCII(out io.Writer, clear bool) *ASCII {
	return &ASCII{out: out, clear: clear}
}

// Frame implements the coordinator's renderer contract.
func (a *ASCII) Frame(tick int, g *gridmap.Grid, pos map[int]gridmap.Coord, status string) {
	var b strings.Builder
	if a.clear {
		b.WriteString("\x1b[2J\x1b[H")
	}
	fmt.Fprintf(&b, "tick %d  %s\n", tick, status)
	b.WriteString(GridString(g, pos))
	fmt.Fprintln(a.out, b.String())
}

// GridString renders the terrain with robot glyphs overlaid. Robots are drawn
// as letters A-Z then digits, ordered by ID.
func GridString(g *gridmap.Grid, pos map[int]gridmap.Coord) string {
	rows := make([][]rune, g.H)
	for y := 0; y < g.H; y++ {
		rows[y] = make([]rune, g.W)
		for x := 0; x < g.W; x++ {
			switch g.At(gridmap.Coord{X: x, Y: y}) {
			case gridmap.Wall:
				rows[y][x] = '#'
			case gridmap.Station:
				rows[y][x] = 'S'
			default:
				rows[y][x] = '.'
			}
		}
	}
	ids := make([]int, 0, len(pos))
	for id := range pos {
		ids = append(ids, id)
	}
	sort.Ints(ids)
	for _, id := range ids {
		c := pos[id]
		if !g.InBounds(c) {
			continue
		}
		rows[c.Y][c.X] = glyph(id)
	}
	var b strings.Builder
	for _, row := range rows {
		b.WriteString(string(row))
		b.WriteByte('\n')
	}
	return b.String()
}

func glyph(id int) rune {
	if id < 26 {
		return rune('A' + id)
	}
	return rune('0' + id%10)
}

// Legend returns a short explanation of the map symbols.
func Legend() string {
	return "# wall  . free  S station  A-Z/0-9 robots"
}
