package gridmap

import (
	"strings"
	"testing"
)

func TestParse(t *testing.T) {
	const src = "" +
		"#####\n" +
		"#S@.#\n" +
		"#.#.#\n" +
		"#..S#\n" +
		"#####\n"
	g, err := Parse(strings.NewReader(src), "inline")
	if err != nil {
		t.Fatalf("Parse: %v", err)
	}
	if g.W != 5 || g.H != 5 {
		t.Fatalf("size = %dx%d, want 5x5", g.W, g.H)
	}
	if len(g.Starts) != 1 || g.Starts[0] != (Coord{2, 1}) {
		t.Fatalf("starts = %v, want [(2,1)]", g.Starts)
	}
	if len(g.Stations) != 2 {
		t.Fatalf("stations = %v, want 2", g.Stations)
	}
	if g.At(Coord{0, 0}) != Wall {
		t.Fatalf("corner not wall")
	}
	if !g.Walkable(Coord{1, 1}) || g.Walkable(Coord{2, 2}) {
		t.Fatalf("walkability wrong")
	}
}

func TestParseRejectsRagged(t *testing.T) {
	if _, err := Parse(strings.NewReader("###\n##\n"), "ragged"); err == nil {
		t.Fatal("expected error for ragged rows")
	}
}

func TestParseRejectsBadChar(t *testing.T) {
	if _, err := Parse(strings.NewReader("###\n#x#\n###\n"), "bad"); err == nil {
		t.Fatal("expected error for unknown character")
	}
}

func TestNeighborsDeterministic(t *testing.T) {
	g := NewGrid(3, 3)
	for y := 0; y < 3; y++ {
		for x := 0; x < 3; x++ {
			g.Set(Coord{x, y}, Free)
		}
	}
	n := g.Neighbors(Coord{1, 1})
	want := []Coord{{1, 0}, {2, 1}, {1, 2}, {0, 1}}
	if len(n) != len(want) {
		t.Fatalf("neighbors = %v, want %v", n, want)
	}
	for i := range want {
		if n[i] != want[i] {
			t.Fatalf("neighbors[%d] = %v, want %v", i, n[i], want[i])
		}
	}
}

func TestGenerateDeterministic(t *testing.T) {
	a := Generate(20, 20, 0.15, 7, 4, 3)
	b := Generate(20, 20, 0.15, 7, 4, 3)
	if len(a.Starts) != len(b.Starts) || len(a.Stations) != len(b.Stations) {
		t.Fatal("Generate not deterministic")
	}
	for i := range a.Starts {
		if a.Starts[i] != b.Starts[i] {
			t.Fatal("start mismatch across identical seeds")
		}
	}
	if len(a.Starts) != 4 || len(a.Stations) != 3 {
		t.Fatalf("starts=%d stations=%d, want 4/3", len(a.Starts), len(a.Stations))
	}
}
