package render

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

// WriteSVG writes a single frame as an SVG file. Each grid cell becomes a
// square of cellSize pixels.
func WriteSVG(path string, tick int, g *gridmap.Grid, pos map[int]gridmap.Coord, cellSize int) error {
	if cellSize <= 0 {
		cellSize = 24
	}
	w := g.W * cellSize
	h := g.H * cellSize
	var b []byte
	b = append(b, []byte(fmt.Sprintf(
		`<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">`+"\n",
		w, h, w, h))...)
	b = append(b, []byte(fmt.Sprintf(`<rect width="%d" height="%d" fill="#111"/>`+"\n", w, h))...)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			c := gridmap.Coord{X: x, Y: y}
			fill := "#222"
			switch g.At(c) {
			case gridmap.Wall:
				fill = "#000"
			case gridmap.Station:
				fill = "#2b6cb0"
			}
			b = append(b, []byte(fmt.Sprintf(
				`<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="#333" stroke-width="1"/>`+"\n",
				x*cellSize, y*cellSize, cellSize, cellSize, fill))...)
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
		b = append(b, []byte(fmt.Sprintf(
			`<circle cx="%d" cy="%d" r="%d" fill="#ecc94b"/>`+"\n"+
				`<text x="%d" y="%d" font-size="%d" text-anchor="middle" fill="#111">%d</text>`+"\n",
			c.X*cellSize+cellSize/2, c.Y*cellSize+cellSize/2, cellSize/3,
			c.X*cellSize+cellSize/2, c.Y*cellSize+cellSize*2/3, cellSize/2, id))...)
	}
	b = append(b, []byte(fmt.Sprintf(`<text x="8" y="%d" fill="#eee" font-size="14">tick %d</text>`+"\n", h-6, tick))...)
	b = append(b, []byte("</svg>\n")...)
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, b, 0o644)
}
