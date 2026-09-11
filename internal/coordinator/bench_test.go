package coordinator

import (
	"strconv"
	"testing"
	"time"

	gridmap "github.com/baasilmirza/robotics-fleet-planner/internal/map"
)

func BenchmarkFleet(b *testing.B) {
	for _, size := range []int{20, 40} {
		for _, robots := range []int{4, 8, 16} {
			name := strconv.Itoa(size) + "x" + strconv.Itoa(size) + "/" + strconv.Itoa(robots) + "robots"
			b.Run(name, func(b *testing.B) {
				for i := 0; i < b.N; i++ {
					g := gridmap.Generate(size, size, 0.08, 3, robots, robots/2+2)
					cfg := Config{
						Grid:              g,
						Robots:            robots,
						Tasks:             robots * 3,
						MaxTicks:          30000,
						Seed:              3,
						TickDuration:      time.Millisecond,
						DeadlockThreshold: 30,
					}
					New(cfg).Run()
				}
			})
		}
	}
}
