package snackrestocker

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

type pickRig struct {
	s         *snackRestockerRestocker
	moves     []r3.Vector
	rotations []float64
	tilts     []float64
	linear    []bool
	grips     []interface{}
	visited   []string
	// refuse, if set, makes the fake planner reject a move to p at angle.
	refuse func(p r3.Vector, angle graspAngle) bool
}

// angleOf returns which rotation, straight down or tilted, a tool orientation at p uses.
func angleOf(p r3.Vector, o spatialmath.Orientation) (graspAngle, bool) {
	for _, tilt := range []float64{0, hoverFallbackTiltDeg} {
		for _, rot := range gripperRotations {
			a := graspAngle{rotationDeg: rot, tiltDeg: tilt}
			if spatialmath.OrientationAlmostEqual(o, toolOrientation(p, a)) {
				return a, true
			}
		}
	}
	return graspAngle{}, false
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
		p := req.Destination.Pose().Point()
		angle, ok := angleOf(p, req.Destination.Pose().Orientation())
		if !ok {
			t.Fatalf("unexpected tool orientation: %+v", req.Destination.Pose().Orientation().OrientationVectorDegrees())
		}
		if rig.refuse != nil && rig.refuse(p, angle) {
			return false, errors.New("all IK solutions failed constraints: violation between bin:bin-left and gripper:claws")
		}
		rig.moves = append(rig.moves, p)
		rig.rotations = append(rig.rotations, angle.rotationDeg)
		rig.tilts = append(rig.tilts, angle.tiltDeg)
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
func segment(conf float64, center, dims r3.Vector) map[string]interface{} {
	return map[string]interface{}{
		"label":           "snack bag",
		"confidence":      conf,
		"reference_frame": "world",
		"center_mm":       vec(center),
		"dims_mm":         vec(dims),
		"max_mm":          vec(center.Add(dims.Mul(0.5))),
	}
}

func twoBags() map[string]interface{} {
	bag := func(conf, x, topZ float64) map[string]interface{} {
		return segment(conf, r3.Vector{X: x, Y: -440, Z: topZ - 30}, r3.Vector{X: 100, Y: 80, Z: 60})
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

// Segments seen on munch-monitor: bag-sized clouds a meter outside the bin,
// stretched clouds from far depth readings, and tiny fragments.
func TestPickSnackSkipsImplausibleSegments(t *testing.T) {
	outsideBin := segment(0.93, r3.Vector{X: 357, Y: -1539, Z: -887}, r3.Vector{X: 360, Y: 245, Z: 152})
	stretched := segment(0.93, r3.Vector{X: 347, Y: -440, Z: 100}, r3.Vector{X: 388, Y: 300, Z: 1409})
	fragment := segment(0.92, r3.Vector{X: 399, Y: -297, Z: 57}, r3.Vector{X: 6, Y: 7, Z: 4})
	realBag := segment(0.90, r3.Vector{X: 121, Y: -419, Z: 181}, r3.Vector{X: 140, Y: 202, Z: 165})
	rig := newPickRig(t, map[string]interface{}{
		"objects": []interface{}{outsideBin, stretched, fragment, realBag},
	})
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "dry_run": true})
	if resp["pick_snack"] != "planned" {
		t.Fatalf("dry run: %v", resp)
	}
	if resp["confidence"] != 0.90 || resp["rejected"] != 3 || resp["candidates"] != 4 {
		t.Fatalf("should skip 3 bad segments and pick the real bag: %v", resp)
	}
	if c := resp["center_mm"].(map[string]interface{}); c["x"] != 121.0 || c["y"] != -419.0 {
		t.Fatalf("center: %v", c)
	}
}

func TestPickSnackFailsWhenNoSegmentIsPlausible(t *testing.T) {
	outsideBin := segment(0.93, r3.Vector{X: 357, Y: -1539, Z: -887}, r3.Vector{X: 360, Y: 245, Z: 152})
	rig := newPickRig(t, map[string]interface{}{"objects": []interface{}{outsideBin}})
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" || len(rig.moves) != 0 || len(rig.grips) != 0 {
		t.Fatalf("should fail without moving toward the bag: %v moves=%v grips=%v", resp, rig.moves, rig.grips)
	}
}

// twoBags puts bag 1 (0.9) at x=250 and bag 2 (0.6) at x=150; tops at z=100
// and z=90, so with the defaults the grab points are z=80 and z=70.

func TestPickSnackRotatesWhenDescendIsRefused(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool {
		return p.Z == 80 && a.rotationDeg == 0 // bag 1's descend hits the wall unrotated
	}
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 1 || resp["rotation_deg"] != 90.0 {
		t.Fatalf("should grab bag 1 rotated 90: %v", resp)
	}
	attempts := resp["attempts"].([]interface{})
	if len(attempts) != 1 {
		t.Fatalf("attempts: %v", attempts)
	}
	a := attempts[0].(map[string]interface{})
	if a["bag"] != 1 || a["rotation_deg"] != 0.0 || a["stage"] != "descend" {
		t.Fatalf("attempt: %v", a)
	}
	// hover 0, hover 90, descend 90, lift 90.
	wantRot := []float64{0, 90, 90, 90}
	if len(rig.rotations) != len(wantRot) {
		t.Fatalf("rotations: %v", rig.rotations)
	}
	for i := range wantRot {
		if rig.rotations[i] != wantRot[i] {
			t.Fatalf("rotations: %v", rig.rotations)
		}
	}
	if len(rig.grips) != 2 || rig.grips[1] != gripperClose {
		t.Fatalf("gripper: %v", rig.grips)
	}
}

func TestPickSnackMovesToNextBagWhenBothRotationsFail(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 } // bag 1 unreachable
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 || resp["rotation_deg"] != 0.0 {
		t.Fatalf("should grab bag 2: %v", resp)
	}
	if c := resp["center_mm"].(map[string]interface{}); c["x"] != 150.0 {
		t.Fatalf("center: %v", c)
	}
	if n := len(resp["attempts"].([]interface{})); n != 2 {
		t.Fatalf("both rotations of bag 1 should be recorded: %v", resp["attempts"])
	}
}

func TestPickSnackReportsAttemptsWhenNothingIsReachable(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z < 100 } // every descend
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "failed" || resp["error"] == nil {
		t.Fatalf("should fail: %v", resp)
	}
	if n := len(resp["attempts"].([]interface{})); n != 4 {
		t.Fatalf("2 bags x 2 rotations should be recorded: %v", resp["attempts"])
	}
	if len(rig.grips) != 1 || rig.grips[0] != gripperOpen {
		t.Fatalf("gripper should only open: %v", rig.grips)
	}
}

func TestPickSnackMaxBags(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "max_bags": 1.0})
	if resp["pick_snack"] != "failed" || resp["plausible"] != 1 {
		t.Fatalf("should only try bag 1: %v", resp)
	}
}

func TestPickSnackHoverOnlyRotatesWhenHoverIsRefused(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return a.rotationDeg == 0 }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true, "hover_only": true})
	if resp["pick_snack"] != "hovering" || resp["bag"] != 1 || resp["rotation_deg"] != 90.0 {
		t.Fatalf("should hover over bag 1 rotated 90: %v", resp)
	}
}

func TestHoverHeightIsCappedNearTheRim(t *testing.T) {
	for _, tc := range []struct{ topZ, approach, want float64 }{
		{100, 120, 220},                 // low bag: full approach
		{356, 120, binRimZ + 50},        // high bag: capped at the rim
		{383, 120, 383 + minApproachMm}, // above the cap: keep minimum clearance
		{383, 20, 403},                  // a smaller approach than the minimum is honored
	} {
		if got := hoverHeight(tc.topZ, tc.approach, defaultHoverAboveRimMm); got != tc.want {
			t.Fatalf("hoverHeight(%v, %v) = %v, want %v", tc.topZ, tc.approach, got, tc.want)
		}
	}
}

// twoBags hovers at z=220 (bag 1) and z=210 (bag 2).
func isHover(p r3.Vector) bool { return p.Z > 200 }

func TestPickSnackTiltsTheHoverOnlyAfterEveryRotationStraightDown(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return isHover(p) && a.tiltDeg == 0 }
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 1 || resp["rotation_deg"] != 0.0 ||
		resp["tilt_deg"] != hoverFallbackTiltDeg {
		t.Fatalf("should grab bag 1 tilted at rotation 0: %v", resp)
	}
	attempts := resp["attempts"].([]interface{})
	if len(attempts) != len(gripperRotations) {
		t.Fatalf("every straight-down hover should be refused first: %v", attempts)
	}
	// Tilted hover, descend and lift all keep the tilt.
	for i, tilt := range rig.tilts {
		if tilt != hoverFallbackTiltDeg {
			t.Fatalf("move %d tilt %v: %v", i, tilt, rig.tilts)
		}
	}
}

func TestPickSnackDoesNotTiltAfterARefusedDescend(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return p.Z == 80 } // bag 1's descend, every rotation
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 {
		t.Fatalf("should move on to bag 2: %v", resp)
	}
	for _, raw := range resp["attempts"].([]interface{}) {
		if a := raw.(map[string]interface{}); a["tilt_deg"] != 0.0 {
			t.Fatalf("a refused descend should not be retried tilted: %v", resp["attempts"])
		}
	}
	for _, tilt := range rig.tilts {
		if tilt != 0 {
			t.Fatalf("no move should be tilted: %v", rig.tilts)
		}
	}
}

func TestPickSnackTriesTiltedHoversBeforeTheNextBag(t *testing.T) {
	rig := newPickRig(t, twoBags())
	rig.refuse = func(p r3.Vector, a graspAngle) bool { return math.Abs(p.Z-220) < 1 } // bag 1's hover, every angle
	resp, _ := rig.s.DoCommand(context.Background(), map[string]interface{}{"pick_snack": true})
	if resp["pick_snack"] != "complete" || resp["bag"] != 2 || resp["tilt_deg"] != 0.0 {
		t.Fatalf("should grab bag 2 straight down: %v", resp)
	}
	var tried []graspAngle
	for _, raw := range resp["attempts"].([]interface{}) {
		a := raw.(map[string]interface{})
		tried = append(tried, graspAngle{rotationDeg: a["rotation_deg"].(float64), tiltDeg: a["tilt_deg"].(float64)})
	}
	want := []graspAngle{{0, 0}, {90, 0}, {0, hoverFallbackTiltDeg}, {90, hoverFallbackTiltDeg}}
	if len(tried) != len(want) {
		t.Fatalf("bag 1 attempts %v, want %v", tried, want)
	}
	for i := range want {
		if tried[i] != want[i] {
			t.Fatalf("bag 1 attempts %v, want %v", tried, want)
		}
	}
}

func TestToolOrientationLeansWristTowardBinCenter(t *testing.T) {
	nearLeftEdge := r3.Vector{X: 250, Y: -280, Z: 200}
	o := toolOrientation(nearLeftEdge, graspAngle{tiltDeg: hoverFallbackTiltDeg})
	if o.OY <= 0 || math.Abs(o.OX) > 1e-9 {
		t.Fatalf("tool should point toward the near wall so the wrist leans in: %+v", o)
	}
	if got := math.Acos(-o.OZ) * 180 / math.Pi; math.Abs(got-hoverFallbackTiltDeg) > 1e-6 {
		t.Fatalf("tilt %v, want %v", got, hoverFallbackTiltDeg)
	}
}
