package snackrestocker

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// Vision-guided pick: a standalone DoCommand, separate from the restock
// sequence. It looks at the bin from restock-1-find, asks the SAM3 segmenter
// for the highest-confidence snack bag, and grabs it from above.

// Optional dependencies used only by pick_snack.
const (
	sam3VisionName = "sam3-vision"
	motionName     = "builtin"
)

// Frame moved by the motion planner: the gripper's tool center point.
const tcpFrame = "gripper-tcp"

// Pose the camera looks at the bin from before segmenting.
const pickViewSwitch = "restock-1-find"

// Defaults, overridable per call.
const (
	defaultApproachMm   = 120.0 // hover height above the top of the bag
	defaultGraspDepthMm = 20.0  // how far below the top of the bag to close
)

// Straight-line tolerance for the descend and lift moves.
var straightDown = &motionplan.Constraints{
	LinearConstraint: []motionplan.LinearConstraint{{LineToleranceMm: 10, OrientationToleranceDegs: 5}},
}

// snackTarget is the best SAM3 segment, in the world frame.
type snackTarget struct {
	label      string
	confidence float64
	center     r3.Vector
	topZ       float64
	candidates int
}

// topDown is the tool pointing straight down (-Z in world).
func topDown(p r3.Vector) *referenceframe.PoseInFrame {
	return referenceframe.NewPoseInFrame(referenceframe.World,
		spatialmath.NewPose(p, &spatialmath.OrientationVectorDegrees{OZ: -1}))
}

func vec(v r3.Vector) map[string]interface{} {
	return map[string]interface{}{"x": v.X, "y": v.Y, "z": v.Z}
}

// findSnack asks sam3-vision for segments (sorted best-first) and returns the
// first one.
func (s *snackRestockerRestocker) findSnack(ctx context.Context) (*snackTarget, error) {
	resp, err := s.vision.DoCommand(ctx, map[string]interface{}{"command": "get_object_point_clouds"})
	if err != nil {
		return nil, fmt.Errorf("%s get_object_point_clouds: %w", sam3VisionName, err)
	}
	if msg, ok := resp["error"]; ok {
		return nil, fmt.Errorf("%s: %v", sam3VisionName, msg)
	}
	objects, _ := resp["objects"].([]interface{})
	if len(objects) == 0 {
		return nil, errors.New("no snack bags found")
	}
	best, ok := objects[0].(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("unexpected object from %s: %v", sam3VisionName, objects[0])
	}
	if frame := fmt.Sprint(best["reference_frame"]); frame != referenceframe.World {
		return nil, fmt.Errorf("snack bag is in frame %q, not world; check the camera's frame config", frame)
	}
	center, err := readVec(best["center_mm"])
	if err != nil {
		return nil, fmt.Errorf("center_mm: %w", err)
	}
	maxes, err := readVec(best["max_mm"])
	if err != nil {
		return nil, fmt.Errorf("max_mm: %w", err)
	}
	confidence, _ := best["confidence"].(float64)
	return &snackTarget{
		label:      fmt.Sprint(best["label"]),
		confidence: confidence,
		center:     center,
		topZ:       maxes.Z,
		candidates: len(objects),
	}, nil
}

func readVec(v interface{}) (r3.Vector, error) {
	m, ok := v.(map[string]interface{})
	if !ok {
		return r3.Vector{}, fmt.Errorf("expected {x, y, z}, got %v", v)
	}
	x, okX := m["x"].(float64)
	y, okY := m["y"].(float64)
	z, okZ := m["z"].(float64)
	if !okX || !okY || !okZ {
		return r3.Vector{}, fmt.Errorf("expected numeric x, y, z, got %v", m)
	}
	return r3.Vector{X: x, Y: y, Z: z}, nil
}

// moveTCP plans and runs a move of the gripper TCP to a world pose.
func (s *snackRestockerRestocker) moveTCP(
	ctx context.Context, desc string, p r3.Vector, constraints *motionplan.Constraints, step int, steps *[]interface{},
) error {
	s.logger.Infof("pick step %d: %s, moving %s to (%.0f, %.0f, %.0f) mm", step, desc, tcpFrame, p.X, p.Y, p.Z)
	if _, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: tcpFrame,
		Destination:   topDown(p),
		Constraints:   constraints,
	}); err != nil {
		return fmt.Errorf("%s: %w", desc, err)
	}
	if err := sleep(ctx, stepDelay); err != nil {
		return err
	}
	*steps = append(*steps, fmt.Sprintf("%s: %s to (%.0f, %.0f, %.0f)", desc, tcpFrame, p.X, p.Y, p.Z))
	return nil
}

// runPick runs pick_snack. dryRun stops after finding the bag; hoverOnly
// stops above it with the gripper open.
func (s *snackRestockerRestocker) runPick(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if s.vision == nil || s.motion == nil {
		return nil, fmt.Errorf("pick_snack needs the '%s' vision service and '%s' motion service on the machine",
			sam3VisionName, motionName)
	}
	dryRun, _ := cmd["dry_run"].(bool)
	hoverOnly, _ := cmd["hover_only"].(bool)
	approachMm := defaultApproachMm
	if v, ok := cmd["approach_mm"].(float64); ok {
		approachMm = v
	}
	graspDepthMm := defaultGraspDepthMm
	if v, ok := cmd["grasp_depth_mm"].(float64); ok {
		graspDepthMm = v
	}

	steps := []interface{}{}
	step := 0
	next := func() int {
		step++
		return step
	}

	// Look at the bin from the first restock pose.
	if err := s.goToPose(ctx, pickViewSwitch, next(), &steps); err != nil {
		return nil, err
	}
	target, err := s.findSnack(ctx)
	if err != nil {
		return nil, err
	}
	hover := r3.Vector{X: target.center.X, Y: target.center.Y, Z: target.topZ + approachMm}
	grasp := r3.Vector{X: target.center.X, Y: target.center.Y, Z: target.topZ - graspDepthMm}
	s.logger.Infof("pick: best of %d is %q (confidence %.2f), center (%.0f, %.0f, %.0f) mm, top z %.0f mm",
		target.candidates, target.label, target.confidence,
		target.center.X, target.center.Y, target.center.Z, target.topZ)

	result := map[string]interface{}{
		"label":      target.label,
		"confidence": target.confidence,
		"candidates": target.candidates,
		"center_mm":  vec(target.center),
		"top_z_mm":   target.topZ,
		"hover_mm":   vec(hover),
		"grasp_mm":   vec(grasp),
		"dry_run":    dryRun,
		"hover_only": hoverOnly,
	}
	finish := func(status string) map[string]interface{} {
		result["pick_snack"] = status
		result["steps"] = steps
		result["steps_completed"] = step
		return result
	}
	if dryRun {
		return finish("planned"), nil
	}

	if err := s.setGripper(ctx, gripperOpen, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.moveTCP(ctx, "hover above snack", hover, nil, next(), &steps); err != nil {
		return nil, err
	}
	if hoverOnly {
		return finish("hovering"), nil
	}
	if err := s.moveTCP(ctx, "descend to snack", grasp, straightDown, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.setGripper(ctx, gripperClose, next(), &steps); err != nil {
		return nil, err
	}
	if err := s.moveTCP(ctx, "lift snack", hover, straightDown, next(), &steps); err != nil {
		return nil, err
	}
	return finish("complete"), nil
}
