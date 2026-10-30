package snackrestocker

import (
	"context"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/testutils/inject"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

type pickRig struct {
	s       *snackRestockerRestocker
	moves   []r3.Vector
	linear  []bool
	grips   []interface{}
	visited []string
}

// newPickRig fakes the switch, gripper, motion and sam3-vision resources and
// records what pick_snack does with them.
func newPickRig(t *testing.T, sam3Resp map[string]interface{}) *pickRig {
	stepDelay = 0
	t.Cleanup(func() { stepDelay = time.Second })
	rig := &pickRig{}

	sw := inject.NewSwitch(pickViewSwitch)
	sw.SetPositionFunc = func(ctx context.Context, position uint32, extra map[string]interface{}) error {
		rig.visited = append(rig.visited, pickViewSwitch)
		return nil
	}
	g := inject.NewGripper(gripperName)
	g.DoFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		rig.grips = append(rig.grips, cmd["set"])
		return nil, nil
	}
	m := injectmotion.NewMotionService(motionName)
	m.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
		if req.ComponentName != tcpFrame {
			t.Fatalf("moved %q, want %q", req.ComponentName, tcpFrame)
		}
		if req.Destination.Parent() != "world" {
			t.Fatalf("destination frame %q, want world", req.Destination.Parent())
		}
		ov := req.Destination.Pose().Orientation().OrientationVectorDegrees()
		if ov.OZ > -0.99 {
			t.Fatalf("tool not pointing down: %+v", ov)
		}
		rig.moves = append(rig.moves, req.Destination.Pose().Point())
		rig.linear = append(rig.linear, req.Constraints != nil && len(req.Constraints.LinearConstraint) > 0)
		return true, nil
	}
	v := inject.NewVisionService(sam3VisionName)
	v.DoCommandFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		if cmd["command"] != "get_object_point_clouds" {
			t.Fatalf("unexpected sam3 command: %v", cmd)
		}
		return sam3Resp, nil
	}

	rig.s = &snackRestockerRestocker{
		logger:     logging.NewTestLogger(t),
		gripper:    g,
		switches:   map[string]toggleswitch.Switch{pickViewSwitch: sw},
		vision:     v,
		motion:     m,
		state:      initialState,
		cancelFunc: func() {},
	}
	return rig
}

// Two bags, best first, as the sam3-segments DoCommand returns them.
func twoBags() map[string]interface{} {
	bag := func(conf, x, topZ float64) map[string]interface{} {
		return map[string]interface{}{
			"label":           "snack bag",
			"confidence":      conf,
			"reference_frame": "world",
			"center_mm":       map[string]interface{}{"x": x, "y": -440.0, "z": topZ - 30},
			"max_mm":          map[string]interface{}{"x": x + 50, "y": -400.0, "z": topZ},
		}
	}
	return map[string]interface{}{"objects": []interface{}{bag(0.9, 250, 100), bag(0.6, 150, 90)}}
}

func TestPickSnackDryRunOnlyGoesToViewPose(t *testing.T) {
	rig := newPickRig(t, twoBags())
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	if resp["pick_snack"] != "planned" {
		t.Fatalf("dry run: %v", resp)
	}
	if len(rig.visited) != 1 || len(rig.moves) != 0 || len(rig.grips) != 0 {
		t.Fatalf("dry run should only visit %s: visited=%v moves=%v grips=%v",
			pickViewSwitch, rig.visited, rig.moves, rig.grips)
	}
	if resp["confidence"] != 0.9 || resp["candidates"] != 2 {
		t.Fatalf("should pick the first (best) bag: %v", resp)
	}
	hover := resp["hover_mm"].(map[string]interface{})
	grasp := resp["grasp_mm"].(map[string]interface{})
	if hover["x"] != 250.0 || hover["z"] != 100+defaultApproachMm || grasp["z"] != 100-defaultGraspDepthMm {
		t.Fatalf("hover=%v grasp=%v", hover, grasp)
	}
}

func TestPickSnackHoverOnly(t *testing.T) {
	rig := newPickRig(t, twoBags())
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{
		"pick_snack": true, "hover_only": true, "approach_mm": 80.0,
	})
	if resp["pick_snack"] != "hovering" {
		t.Fatalf("hover: %v", resp)
	}
	if len(rig.moves) != 1 || rig.moves[0] != (r3.Vector{X: 250, Y: -440, Z: 180}) {
		t.Fatalf("moves: %v", rig.moves)
	}
	if len(rig.grips) != 1 || rig.grips[0] != gripperOpen {
		t.Fatalf("gripper should only open: %v", rig.grips)
	}
}

func TestPickSnackFullGrab(t *testing.T) {
	rig := newPickRig(t, twoBags())
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" {
		t.Fatalf("pick: %v", resp)
	}
	want := []r3.Vector{{X: 250, Y: -440, Z: 220}, {X: 250, Y: -440, Z: 80}, {X: 250, Y: -440, Z: 220}}
	if len(rig.moves) != len(want) {
		t.Fatalf("moves: %v", rig.moves)
	}
	for i := range want {
		if rig.moves[i] != want[i] {
			t.Fatalf("move %d: got %v want %v", i, rig.moves[i], want[i])
		}
	}
	if rig.linear[0] || !rig.linear[1] || !rig.linear[2] {
		t.Fatalf("descend and lift should be straight-line: %v", rig.linear)
	}
	if len(rig.grips) != 2 || rig.grips[0] != gripperOpen || rig.grips[1] != gripperClose {
		t.Fatalf("gripper: %v", rig.grips)
	}
	if rig.s.currentState() != initialState {
		t.Fatalf("pick_snack must not touch the restock state machine: %s", rig.s.currentState())
	}
}

func TestPickSnackErrors(t *testing.T) {
	cases := map[string]map[string]interface{}{
		"no bags":   {"objects": []interface{}{}},
		"sam3 err":  {"error": "unknown command: get_object_point_clouds"},
		"cam frame": {"objects": []interface{}{map[string]interface{}{"reference_frame": "cam"}}},
	}
	for name, sam3Resp := range cases {
		rig := newPickRig(t, sam3Resp)
		resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
		if resp["pick_snack"] != "failed" || resp["error"] == nil {
			t.Fatalf("%s: %v", name, resp)
		}
		if len(rig.moves) != 0 || len(rig.grips) != 0 {
			t.Fatalf("%s: must not move toward a bad target: moves=%v grips=%v", name, rig.moves, rig.grips)
		}
	}
}

func TestPickSnackWithoutServices(t *testing.T) {
	s := &snackRestockerRestocker{logger: logging.NewTestLogger(t), state: initialState, cancelFunc: func() {}}
	resp, _ := s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" {
		t.Fatalf("missing services: %v", resp)
	}
}
